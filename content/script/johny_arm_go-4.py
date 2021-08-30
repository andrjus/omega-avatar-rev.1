import serial
import json
import time
ser = serial.Serial('COM4', 115200, timeout=0, rtscts=0)

print(ser)
					
hand_commands = [
    {
        "command": "ARM",
        "timestamp": 1619009891000,
        "arm": 0,
        "pos": {
            "x": -200.0,
            "y": 400.0,
            "z": 470.0
        },
        "rot": {
            "r": -0.0,
            "p": -0.0,
            "y": -0.0
        },
        "fingers": [40]
    }
]
j_str = json.dumps(hand_commands[0]).replace(" ", "")
j_str += '\n'
b_str = bytes(j_str.encode('ascii'))
ser.write(b_str)     # write a string
time.sleep(1)
hand_commands = [
    {
        "command": "ARM",
        "timestamp": 1619009891000,
        "arm": 0,
        "pos": {
            "x": -200.0,
            "y": 400.0,
            "z": 100.0
        },
        "rot": {
            "r": -0.0,
            "p": -0.0,
            "y": -1.0
        },
        "fingers": [40]
    }
]
j_str = json.dumps(hand_commands[0]).replace(" ", "")
j_str += '\n'
b_str = bytes(j_str.encode('ascii'))
ser.write(b_str)     # write a string
time.sleep(1)
hand_commands = [
    {
        "command": "ARM",
        "timestamp": 1619009891000,
        "arm": 0,
        "pos": {
            "x": -150.0,
            "y": 50.0,
            "z": 100.0
        },
        "rot": {
            "r": -0.0,
            "p": -1.0,
            "y": -2.0
        },
        "fingers": [40]
    }
]
j_str = json.dumps(hand_commands[0]).replace(" ", "")
j_str += '\n'
b_str = bytes(j_str.encode('ascii'))
ser.write(b_str)     # write a string
time.sleep(5)

j_str = json.dumps(hand_commands[0]).replace(" ", "")
j_str += '\n'
b_str = bytes(j_str.encode('ascii'))
ser.write(b_str)     # write a string
time.sleep(2)
hand_commands = [
    {
        "command": "ARM",
        "timestamp": 1619009891000,
        "arm": 0,
        "pos": {
            "x": -150.0,
            "y": 350.0,
            "z": 470.0
        },
        "rot": {
            "r": -0.0,
            "p": -0.0,
            "y": -0.0
        },
        "fingers": [40]
    }
]
j_str = json.dumps(hand_commands[0]).replace(" ", "")
j_str += '\n'
b_str = bytes(j_str.encode('ascii'))
ser.write(b_str)     # write a string
time.sleep(2)

hand_commands = [
    {
        "command": "ARM",
        "timestamp": 1619009891000,
        "arm": 0,
        "pos": {
            "x": -150.0,
            "y": 350.0,
            "z": 470.0
        },
        "rot": {
            "r": -0.0,
            "p": -0.0,
            "y": -0.0
        },
        "fingers": [100]
    }
]
j_str = json.dumps(hand_commands[0]).replace(" ", "")
j_str += '\n'
b_str = bytes(j_str.encode('ascii'))
ser.write(b_str)     # write a string
time.sleep(2)

hand_commands = [
    {
        "command": "ARM",
        "timestamp": 1619009891000,
        "arm": 0,
        "pos": {
            "x": -150.0,
            "y": 350.0,
            "z": 470.0
        },
        "rot": {
            "r": -0.0,
            "p": -0.0,
            "y": -0.0
        },
        "fingers": [100]
    }
]
j_str = json.dumps(hand_commands[0]).replace(" ", "")
j_str += '\n'
b_str = bytes(j_str.encode('ascii'))
ser.write(b_str)     # write a string
time.sleep(1)

ser.close()  
					
					