import serial
import json
import time
ser = serial.Serial('COM4', 9600, timeout=0, rtscts=0)

print(ser)
					
hand_commands = [
    {
        "command": "CALIBRATE",
    }
]
j_str = json.dumps(hand_commands[0]).replace(" ", "")
j_str=j_str+'\n'
b_str = bytes(j_str.encode('ascii'))
ser.write(b_str)     # write a string
time.sleep(1)
ser.close()  
					
					