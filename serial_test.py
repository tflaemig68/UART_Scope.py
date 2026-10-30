import serial

ser = serial.Serial("COM8", 128000)

while True:
    print(ser.read(1))