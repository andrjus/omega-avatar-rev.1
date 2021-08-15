import serial
import json
import time
ser = serial.Serial('COM11', 9600, timeout=0, rtscts=1)

print(ser)
					
hand_commands = [
    {
        "command": "REBOOT",
    }
]
j_str = json.dumps(hand_commands[0]).replace(" ", "")
j_str += '\n'
b_str = bytes(j_str.encode('ascii'))
ser.write(b_str)     # write a string
time.sleep(0.1)
ser.close()  
					
					