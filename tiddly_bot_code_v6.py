import RPi.GPIO as GPIO
from RPIO import PWM
from random import randint
import time
import curses
import sys

## set which pins are operating the motors and the line sensor
left_motor = 17
right_motor = 27
camera_servo = 22
pen_servo = 9
line_sensor = 23

##define LED pins
led_one = 2
led_two = 3
led_three = 4

## define button pin
button_pin = 24

## Setup GPIO for reading the line sensor and LED's
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO. setup(button_pin, GPIO.IN)
GPIO.setup(line_sensor, GPIO.IN)
GPIO.setup(led_one, GPIO.OUT)
GPIO.setup(led_two, GPIO.OUT)
GPIO.setup(led_three, GPIO.OUT)
## Default LED's to off
GPIO.output(led_one, False)
GPIO.output(led_two, False)
GPIO.output(led_three, False)

## Set up PWM for servos
servo = PWM.Servo()

## Variables for smooth servo rotation
target_left = 1520
target_right = 1520
last_frame = 0

## Display output to console
## Upto 5 lines to output, must have oneline
def display_input(stdscr, out1, out2="", out3="", out4="", out5="", out6=""): 
    stdscr.clear()
    stdscr.addstr(out1)
    stdscr.move(1, 0)
    stdscr.addstr(out2)
    stdscr.move(2, 0)
    stdscr.addstr(out3)
    stdscr.move(3, 0)
    stdscr.addstr(out4)
    stdscr.move(4, 0)
    stdscr.addstr(out5)
    stdscr.move(5, 0)
    stdscr.addstr(out6)
    stdscr.move(23, 79)
    stdscr.refresh()
        

def main(stdscr):
    ## Time vars
    start_time = time.time()
    tenth_seconds = 0
    seconds = 0
    tri_seconds = 0
    time_change_point_1 = 0
    time_change_1 = 0
    time_change_3 = 0
    last_frame = time.time()*1000
    ## Light Vars
    led_1_on = False
    led_2_on = False
    led_3_on = False
    led_light_show = False
    led_counter = 0
    ## Movement Vars
    random_movement = False
    current_left = 1520
    current_right = 1520
    ## Default camera position
    state = 1500
    ## Camera step difference
    step = 100
    ## Default movement vector str
    movement_vector_str = "Stopped"
    k = 0
    in_line_following = False
    try:
        stdscr.nodelay(1)
        while True:
            delta, last_frame = get_delta(last_frame)  # Get time taken to do last loop
            ## get user input
            c = stdscr.getch()
            ## if no input and not in line following
            if (c != -1) and (in_line_following is False):
                if (c == 10) and (c != k):  # 'Return' run custom code
                    k = c
                    movement_vector_str = "Your custom code is running!"
                    custom_code()
                elif (c == 261)| (c == 100) and (c != k):  # 'Right' or 'D' key turn right
                    k = c
                    set_target_speed(0, 80)  
                    movement_vector_str = "Right"
                elif (c == 260) | (c == 97) and (c != k):  # 'Left' or 'A' key turn left
                    k = c
                    set_target_speed(80, 0)
                    movement_vector_str = "Left"
                elif (c == 259) | (c == 119) and (c != k):  # 'Up' or 'W' key turn forwards
                    k = c
                    set_target_speed(100, 100)
                    movement_vector_str = "Forwards"
                elif (c == 258) | (c == 115) and (c != k):  # 'Down' or 'S' key turn backwards
                    k = c
                    set_target_speed(-100, -100)
                    movement_vector_str = "Backwards"
                elif (c == 120) and (c != k):  # 'X' key stop movement
                    k = c
                    set_target_speed(0, 0)
                    movement_vector_str = "Stopped"
                elif (c == 32) and (c != k):  # Switch to line following 'Space'
                    k = c
                    set_target_speed(0, 0)
                    in_line_following = True
                    line_following()  # Run line follower code
                    movement_vector_str = "Line Following"
                elif c == 102 and (c != k):  # Rotate pen down
                    k = c
                    pen_movement("up")
                    movement_vector_str = "Pen up"
                elif c == 103 and c != k:  # Rotate pen up key 'F'
                    k = c
                    pen_movement("down")
                    movement_vector_str = "Pen down"
                elif c == 101:  # camera up - 'E'
                    k = c
                    state = camera_movement("up", step, state)
                    movement_vector_str = "Camera up " + str(state)
                elif c == 114:  # camera down - 'R'
                    k = c
                    state = camera_movement("down", step, state)
                    movement_vector_str = "Camera down " + str(state)
                elif c == 49:  # Led one toggle on or off
                    led_1_on = toggle_led(led_1_on, led_one)
                    led_light_show = False
                elif c == 50:  # Led two toggle on or off
                    led_2_on = toggle_led(led_2_on, led_two)
                    led_light_show = False
                elif c == 51:  # Led three toggle on or off
                    led_3_on = toggle_led(led_3_on, led_three)
                    led_light_show = False
                elif c == 52:  # Allow Led light show to happen
                    if led_light_show == True:
                        led_light_show = False
                        set_led(0,0,0)
                    else:
                        led_light_show = True
                elif (c == 113):  # Quit script 'Q'
                    display_input(stdscr, "Quiting TiddlyBot Script!")
                    clean_up()
                else:  #  Keys not mapped
                    if k != c:
                        movement_vector_str = "ASCII " + str(c) + " Value does nothing!"
            ## line following button actions
            elif c == 32:
                in_line_following = False
                set_target_speed(0, 0)  # stop wheels turning
                movement_vector_str = "Stopped"
            elif (c == 113):  # Quit script 'Q'
                display_input(stdscr, "Quiting TiddlyBot Script!")
                clean_up()
            else:
                if button_pressed():
                    random_movement = True
                    movement_vector_str = "Random Move Mode!"
                if in_line_following:
                    line_following()  # run line follower code
                    movement_vector_str = "Line Following"
            ## every loop, execute this logic
            time_change_point_1 = int((time.time() - start_time)/0.1)
            if time_change_point_1 > tenth_seconds:  # Every 10th a second
                tenth_seconds = time_change_point_1
                ## Update movement servo positions
                current_left = update_servos(current_left, target_left, left_motor)
                current_right = update_servos(current_right, target_right, right_motor)
                
            
            time_change_1 = int(time.time() - start_time)
            if time_change_1 > seconds:  # Every 10th a second
                seconds = time_change_1
                if led_light_show:  # Change LED colour every second
                    if int(round(led_counter)) == 6: 
                        led_counter = 0
                    led_counter += 1
                    led_display(int(round(led_counter)))
            
            time_change_3 = int((time.time() - start_time)/3)
            if time_change_3 > tri_seconds:  # Every 10th a second
                tri_seconds = time_change_3
                if random_movement:  # Random movement ever 3 seconds
                    random_movements()
                
            display_input(stdscr, "Movement: " + movement_vector_str, 
            "Left Speed: " + str(-(current_left-1520)) + "  Right Speed: " + str(current_right -1520),
            "Delta: " + str(delta) + "ms",
            "Seconds: " + str(tenth_seconds/10.0),
            "Tri-seconds: " + str(tri_seconds * 3))            
    except curses.error:
        raise
        
    
    # Write your code for TiddlyBot here    
    
def custom_code():
    pass


    ## Logic for TiddlyBot
    
def button_pressed():
    if GPIO.input(button_pin) == GPIO.HIGH:
        time.sleep(0.01)  # debounce protection
        if GPIO.input(button_pin) == GPIO.HIGH:
            return True
        else:
            return False
    else:
        return False
        
    
def random_movements():
    turn_movement_servos(((randint(0, 20)*10)-100), ((randint(0, 10)*20)-100))


def set_target_speed(left, right):  # Set target speed to achieve
    global target_left, target_right
    target_left = 1520 - left
    target_right = 1520 + right


def update_servos(current, target, motor):  # Set servos toward target at  fixed rate
    direction = target - current
    if current == target:
        return current
    direction = direction_correcting(direction)
    current = smooth_movement_servos(motor, current, direction)
    return current
    

def direction_correcting(direction):  # Take different in target and current to get direction
    if direction > 0:
        direction = 1
    elif direction < 0:
        direction = -1
    else:
        direction = 0
    return direction
    

def clean_up():  # Clean up and exit script
    set_led(0,0,0)
    GPIO.cleanup()
    PWM.cleanup()
    time.sleep(0.5)
    sys.exit()


    ## Wheel servo functions

def turn_movement_servos(left_speed, right_speed):  # Hard setting of left and right wheel servos
    global target_left, target_right
    if left_speed == 0:
        pass
        #servo.stop_servo(left_motor)
    else:
        target_left = 1520 + (-1 * left_speed)
        servo.set_servo(left_motor, target_left)
    if right_speed == 0:
        pass
        #servo.stop_servo(right_motor)
    else:
        target_right = 1520 + (1 * right_speed)
        servo.set_servo(right_motor, target_right)


def smooth_movement_servos(motor, pwm, direction):  # Set a servo with small increment (direction), return new pwm
    pwm += direction * 10
    servo.set_servo(motor, pwm)
    return pwm


    ## Line following functions

def check_on_line():  # Check if TiddlyBot is on a line or not
    if GPIO.input(line_sensor) == GPIO.LOW:
        on_line = True
    else:
        on_line = False
    return on_line


def line_following():  # Basic line following algorithm
    if check_on_line():
        turn_movement_servos(100, 40)
    else:
        turn_movement_servos(40, 100)
    time.sleep(0.1)
    
    
    ## Led functions
    
def toggle_led(led_on, led_pin):
    if led_on:
        GPIO.output(led_pin, False)
        led_on = not led_on
    else:
        GPIO.output(led_pin, True)
        led_on = not led_on
    return led_on


def led_display(led_counter):
    if led_counter == 1:
        set_led(1,0,0)  # red
    elif led_counter == 2:
        set_led(0,1,0)  # green
    elif led_counter == 3:
        set_led(0,0,1)   # blue
    elif led_counter == 4:
        set_led(1,1,0)   # yellow
    elif led_counter == 5:
        set_led(1,0,1)  # indigo
    elif led_counter == 6:
        set_led(0,1,1)  # cyan
        
        
def set_led(one, two, three):
    GPIO.output(led_one, one)
    GPIO.output(led_two, two)
    GPIO.output(led_three, three)
    

    ## Extra movement servo functions

def pen_movement(position):  # Move pen servo up or down, takes a string
    if position == "up":
        servo.set_servo(pen_servo, 2200)
    elif position == "down":
        servo.set_servo(pen_servo, 600)


def camera_movement(direction, step, state):  # Move camera to different positions in steps
    if direction is "up":
        if (state - step) > 700:
            state -= step
        elif (state - step) <= 700:
            state = 700
    elif direction is "down":
        if (state + step) < 2400:
            state += step
        elif (state + step) >= 2400:
            state = 2400
    servo.set_servo(camera_servo, state)
    return state
    
    
    ## Time functions    
    
def get_delta(last_frame):
    current_time = time.time()*1000
    delta = (int(current_time - last_frame))
    last_frame = time.time()*1000
    return delta, last_frame


    ## Main loop in this function
curses.wrapper(main)