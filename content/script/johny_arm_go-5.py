import serial
import json
import time
ser = serial.Serial('COM4', 9600, timeout=0, rtscts=0)

print(ser)
					
hand_commands = [
    {
        "command": "ARM",
        "timestamp": 1619009891000,
        "arm": 0,
        "pos": {
            "x": 100.0,
            "y": 450.0,
            "z": 250.0
        },
        "rot": {
            "r": 0,
            "p": -45.0,
            "y": -75.0
        },
        "fingers": [100]
    }
]
j_str = json.dumps(hand_commands[0]).replace(" ", "")
j_str += '\n'
b_str = bytes(j_str.encode('ascii'))
ser.write(b_str)     # write a string
time.sleep(0.1)
ser.close()  
					
					