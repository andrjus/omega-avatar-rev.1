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
            "x": 100,
            "y": 500,
            "z": 10
        },
        "rot": {
            "r": -0.0,
            "p": -85.0,
            "y": -90.0
        },
        "fingers": [0]
    }
]
j_str = json.dumps(hand_commands[0]).replace(" ", "")
j_str += '\n'
b_str = bytes(j_str.encode('ascii'))
ser.write(b_str)     # write a string
time.sleep(0.1)
ser.close()  
					
				