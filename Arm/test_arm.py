import time
import threading
import queue
import time

from adafruit_servokit import ServoKit

def read_kbd_input(inputQueue):
    print('Ready for keyboard input:')
    while (True):
        # Receive keyboard input from user.
        input_str = input()
        
        # Enqueue this input string.
        # Note: Lock not required here since we are only calling a single Queue method, not a sequence of them 
        # which would otherwise need to be treated as one atomic operation.
        inputQueue.put(input_str)

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

def go_forward(angle=0.2):
    kit.continuous_servo[1].throttle = 1
    kit.continuous_servo[2].throttle = -1
        
    # rotate the servos
    time.sleep(abs(angle))
    
    # stop the servos
    for id_servo in [1,2]:
        kit.continuous_servo[id_servo].throttle = 0


def go_backward(angle=0.2):
    kit.continuous_servo[1].throttle = -1
    kit.continuous_servo[2].throttle = 1
        
    # rotate the servos
    time.sleep(abs(angle))
    
    # stop the servos
    for id_servo in [1,2]:
        kit.continuous_servo[id_servo].throttle = 0

def main():

    EXIT_COMMAND = "exit" # Command to exit this program

    # The following threading lock is required only if you need to enforce atomic access to a chunk of multiple queue
    # method calls in a row.  Use this if you have such a need, as follows:
    # 1. Pass queueLock as an input parameter to whichever function requires it.
    # 2. Call queueLock.acquire() to obtain the lock.
    # 3. Do your series of queue calls which need to be treated as one big atomic operation, such as calling
    # inputQueue.qsize(), followed by inputQueue.put(), for example.
    # 4. Call queueLock.release() to release the lock.
    # queueLock = threading.Lock() 

    #Keyboard input queue to pass data from the thread reading the keyboard inputs to the main thread.
    inputQueue = queue.Queue()

    # Create & start a thread to read keyboard inputs.
    # Set daemon to True to auto-kill this thread when all other non-daemonic threads are exited. This is desired since
    # this thread has no cleanup to do, which would otherwise require a more graceful approach to clean up then exit.
    inputThread = threading.Thread(target=read_kbd_input, args=(inputQueue,), daemon=True)
    inputThread.start()

    # Main loop
    while (True):

        # Read keyboard inputs
        # Note: if this queue were being read in multiple places we would need to use the queueLock above to ensure
        # multi-method-call atomic access. Since this is the only place we are removing from the queue, however, in this
        # example program, no locks are required.
        if (inputQueue.qsize() > 0):
            input_str = inputQueue.get()
            #print("input_str = {}".format(input_str))

            if (input_str == EXIT_COMMAND):
                print("Exiting serial terminal.")
                break # exit the while loop
            
            # Insert your code here to do whatever you want with the input_str.
            angle = float(input_str)
            change_servo(angle)

        # The rest of your program goes here.

        # Sleep for a short time to prevent this thread from sucking up all of your CPU resources on your PC.
        time.sleep(0.01) 
    
    print("End.")


# If you run this Python file directly (ex: via `python3 this_filename.py`), do the following:
if (__name__ == '__main__'): 
    main()
