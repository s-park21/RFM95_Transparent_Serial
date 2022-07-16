import time
import serial

# configure the serial connections (the parameters differs on the device you are connecting to)
ser = serial.Serial(
    port='COM5',
    baudrate=57600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS
)
i = list(range(97,123))
while 1 :
    # ser.write(i)
    # print(i)
    while ser.inWaiting() > 0:
        b = ser.read(1)
        # print(b, end = '')
        print(int.from_bytes(b, "big"),  end = '')
        print(" ",  end = '')

