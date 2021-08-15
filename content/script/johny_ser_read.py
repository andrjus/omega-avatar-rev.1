import serial
import time

#ser = serial.Serial('COM105', 38400, timeout=0,
#                    parity=serial.PARITY_EVEN, rtscts=1)


with serial.Serial('COM16', 9600,
    timeout=10, rtscts=1) as ser:
    print(ser)
    while True:
        line = ser.readline()
        print(line)
#    time.sleep(1)
    