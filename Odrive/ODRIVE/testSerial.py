import serial 
import time 

ser = serial.Serial(port = 'COM17', baudrate=115200,timeout=1)

ser.write(b"r axis0.encoder.pos_estimate\n")
print(ser.readline().decode())