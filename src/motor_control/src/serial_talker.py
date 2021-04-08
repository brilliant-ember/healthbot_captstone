import serial
from time import sleep

COM = '/dev/ttyUSB0'
BAUD = 115200

ser = serial.Serial(COM, BAUD, timeout=.1)

print('Waiting for device')
sleep(1)
print(ser.name)


while True:
    try:
        ser.write(b"blue\n")
        val = str(ser.readline().decode().strip('\r\n'))
        val = val.split()
        if (len(val) == 5):
            # distance in micrometers, velocity in micrometer/milisec
            distance_right_wheel = val[1]
            distance_left_wheel = val[2]
            vel_right = val[3]
            vel_left = val[4]
        # print(dx, end="\n")
        print(val, end="\n")
    except(UnicodeDecodeError):
        pass
