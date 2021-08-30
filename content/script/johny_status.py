import serial
import json
import time
ser = serial.Serial('COM4', 115200, timeout=1, rtscts=0)

print(ser)
					
hand_commands = [
    {
        "command": "STATUS",
        "timestamp": "1619009891000"
    }
]
j_str = json.dumps(hand_commands[0]).replace(" ", "")
j_str=j_str+'\n'
b_str = bytes(j_str.encode('ascii'))
ser.write(b_str)     # write a string
line = ser.readline()
print(line)
ser.close()  
					
					