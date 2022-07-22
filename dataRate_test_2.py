import time
import serial

# configure the serial connections (the parameters differs on the device you are connecting to)
ser1 = serial.Serial(
    port='COM9',
    baudrate=57600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS
)
ser2 = serial.Serial(
    port='COM5',
    baudrate=57600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS
)
serf1 = open("./ser1.txt", "w")
serf2 = open("./ser2.txt", "w")
while 1 :
    while ser1.inWaiting() > 0:
        serf1.write(str(int.from_bytes(ser1.read(1), "big")))
    while ser2.inWaiting() > 0:
        serf2.write(str(int.from_bytes(ser2.read(1), "big")))
