import serial
import numpy as np
from time import time, sleep
import keyboard

def initialize_encoders(com='COM6', baudrate=115200, timeout=1, num_init_loops=5):
    print("Beginnining Initialization")
    arduino = serial.Serial(com,baudrate,timeout=timeout)
    sleep(0.1)
    arduino.reset_input_buffer()
    for i in range(num_init_loops):
        try:
            get_reading(arduino)
        except:
            print("Error Initializing... Retrying")
            pass
    return arduino
   
def decode_reading(reading, num_encoders):
    encoder_output = np.zeros(num_encoders)
    string_list = reading.decode("utf-8").split(",")
    encoder_output[0] = float(string_list[0])*(2*np.pi/4096)
    encoder_output[1] = float(string_list[1])*(2*np.pi/4096)
    return encoder_output

def get_reading(arduino):
    reading = 'empty_reading'
    while reading == 'empty_reading':
        arduino.write(b'b') # send any byte to call reading, sending 'b'
        reading = arduino.readline()
    return reading

def get_encoder_feedback(arduino, num_encoders=2, joint_offsets=np.array([3.15846644+np.pi/2 - 1.60147594, 4.73233073 - 1.53551477])):
    # joint_offsets - set in radians to change x/y position
    reading = get_reading(arduino)
    theta = decode_reading(reading, num_encoders=num_encoders)
    theta -= joint_offsets
    theta *= -1
    return theta

def save_data(output):
    np.savetxt("csv/EncoderTest2.csv", np.array(output), delimiter=",")
    print("Data saved")
    
# to test functions:
if __name__ == "__main__":
    arduino = initialize_encoders()
    output = []
    t0 = time()
    while True:
        theta = get_encoder_feedback(arduino, num_encoders=2)
        print(theta)
        t = time()-t0
        output += [[t,theta[0],theta[1]]]

        if keyboard.is_pressed('esc'):
            save_data(output)
            print("Stopping: User input stop command")
            break