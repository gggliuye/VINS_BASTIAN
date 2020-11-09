import time
import threading
import queue
import time
import curses

from adafruit_servokit import ServoKit

# initialize the PCA9685 system (which is a 16 channel servo system)
kit = ServoKit(channels=16)
# servo for a standard servo (angle)
# continuous_servo for continuous rotating servo (throttle)

#import Adafruit_PCA9685
#pwm = Adafruit_PCA9685.PCA9685(address=0x41, busnum=1)

def change_servo(angle, ids=[1]):
    print("  Rotate to %f"%(angle))
    direction = -1
    if angle > 0:
        direction = 1
    for id_servo in ids:
        kit.continuous_servo[id_servo].throttle = direction
        
    # rotate the servos
    time.sleep(abs(angle))
    
    # stop the servos
    for id_servo in ids:
        kit.continuous_servo[id_servo].throttle = 0

magnitude = 0.15

def go_forward(angle=magnitude):
    kit.continuous_servo[1].throttle = 1
    # rotate the servos
    time.sleep(abs(angle)*0.8)
    kit.continuous_servo[1].throttle = 0


def go_backward(angle=magnitude):
    kit.continuous_servo[1].throttle = -1        
    # rotate the servos
    time.sleep(abs(angle))
    kit.continuous_servo[1].throttle = 0

def go_left(angle=magnitude, direction=1):
    kit.continuous_servo[0].throttle = direction
    # rotate the servos
    time.sleep(abs(angle))
    kit.continuous_servo[0].throttle = 0
    
def hand(angle=magnitude, direction=1):
    kit.continuous_servo[3].throttle = direction
    # rotate the servos
    time.sleep(abs(angle))
    kit.continuous_servo[3].throttle = 0

def go_up(angle=magnitude, direction=1):
    kit.continuous_servo[2].throttle = direction
    # rotate the servos
    if direction:
        time.sleep(abs(angle)*2)
    else:
        time.sleep(abs(angle))
    kit.continuous_servo[2].throttle = 0

def input_char(message):
    try:
        win = curses.initscr()
        win.addstr(0, 0, message)
        while True: 
            ch = win.getch()
            if ch in range(32, 127): 
                break
            time.sleep(0.05)
    finally:
        curses.endwin()
    return chr(ch)

def main():

    EXIT_COMMAND = 'q' # Command to exit this program

    # Main loop
    while (True):

        c = input_char('\n input commond ... \n')
        if c.lower() == EXIT_COMMAND:
            break;

        # The rest of your program goes here.
        if c.lower() in ['w', '8']:
            go_forward()
        elif c.lower() in ['s', '2']:
            go_backward()
        elif c.lower() in ['a', '4']:
            go_left(direction=1)
        elif c.lower() in ['d', '6']:
            go_left(direction=-1)
        elif c.lower() in ['q', '7']:
            go_up(direction=1)
        elif c.lower() in ['e', '9']:
            go_up(direction=-1)
        elif c.lower() in ['5']:
            hand(direction=1)
        elif c.lower() in ['0']:
            hand(direction=-1)

        # Sleep for a short time to prevent this thread from sucking up all of your CPU resources on your PC.
        time.sleep(0.01) 
    
    print("End.")


# If you run this Python file directly (ex: via `python3 this_filename.py`), do the following:
if (__name__ == '__main__'): 
    main()
