import time
import serial

# configure the serial connections (the parameters differs on the device you are connecting to)
ser = serial.Serial(
    port='COM3',
    baudrate=57600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS
)
i = list(range(0,255))
while 1 :
    ser.write(i)
    print(i)
    while ser.inWaiting() > 0:
        print(ser.read(1))
    time.sleep(2)
