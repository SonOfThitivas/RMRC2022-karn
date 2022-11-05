# Importing Libraries
import keyboard as kb
import serial
import time
arduino = serial.Serial(port='COM7', baudrate=9600, timeout=.1)

def write_read(x):
    arduino.write(bytes(x, 'utf-8'))
    time.sleep(0.001)
    data = arduino.readline()
    return data

while True:
    # Car Movement
    
    if kb.is_pressed("w"):
        print("forward")
        write_read("1")
    elif kb.is_pressed("s"):
        print("backward")
        write_read('2')
    elif kb.is_pressed("a"):
    
        print("left")
        write_read('3')
    elif kb.is_pressed("d"):
        print("right")
        write_read('4')
    
    # Servo select    
    elif kb.is_pressed("1"):
        print("Servo1")
        write_read('5')
    elif kb.is_pressed("2"):
        print("Servo2")
        write_read('6')
    elif kb.is_pressed("3"):
        print("Servo3")
        write_read('7')

    # Servo movement
    elif kb.is_pressed("right"):
        print("increase angle")
        write_read("8") 
    elif kb.is_pressed("left"):
        print("decrease angle")
        write_read('9')
    # Break
    elif kb.is_pressed("q"):
        print("break")
        write_read('10')
    else:
        print("stop")
        write_read('0')
    #time.sleep(0.001)