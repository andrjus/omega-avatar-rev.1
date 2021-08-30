import serial
import json
import time
ser = serial.Serial('COM4', 115200, timeout=0, rtscts=1)

print(ser)
					
while True:
    hand_commands = [
        {
            "command": "ARM",
            "timestamp": 1619009891000,
            "arm": 0,
            "pos": {
                "x": 500,
                "y": 0,
                "z": 150
            },
            "rot": {
                "r": 0.0,
                "p": 0.0,
                "y": 0.0
            },
            "fingers": [60]
        }
    ]
    j_str = json.dumps(hand_commands[0]).replace(" ", "")
    b_str = bytes(j_str.encode('ascii'))
    ser.write(b_str)     # write a string
    line = ser.readline()
    print(line)
    time.sleep(5)
    hand_commands = [
        {
            "command": "ARM",
            "timestamp": 1619009891000,
            "arm": 0,
            "pos": {
                "x": 0.0,
                "y": 500,
                "z": 350
            },
            "rot": {
                "r": 0.0,
                "p": 0.0,
                "y": 70.0
            },
            "fingers": [60]
        }
    ]
    j_str = json.dumps(hand_commands[0]).replace(" ", "")
    b_str = bytes(j_str.encode('ascii'))
    ser.write(b_str)     # write a string
    line = ser.readline()
    print(line)
    time.sleep(5)
ser.close()  
					
					