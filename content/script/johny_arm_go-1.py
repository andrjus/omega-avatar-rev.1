import serial
import json
import time
ser = serial.Serial('COM11', 9600, timeout=0, rtscts=1)

print(ser)
					
hand_commands = [
    {
        "command": "ARM",
        "timestamp": 1619009891000,
        "arm": 0,
        "pos": {
            "x": -600.,
            "y": 0,
            "z": 400
        },
        "rot": {
            "r": -0.0,
            "p": 0.0,
            "y": -0.0
        },
        "fingers": [255]
    }
]
j_str = json.dumps(hand_commands[0]).replace(" ", "")
j_str += '\n'
b_str = bytes(j_str.encode('ascii'))
ser.write(b_str)     # write a string
time.sleep(0.1)
ser.close()  
					
					