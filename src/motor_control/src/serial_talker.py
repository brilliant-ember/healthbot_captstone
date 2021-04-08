import serial
from time import sleep

COM = '/dev/ttyUSB0'
BAUD = 115200

ser = serial.Serial(COM, BAUD, timeout = .1)

print('Waiting for device');
sleep(1)
print(ser.name)


while True:
	try:
		val = str(ser.readline().decode().strip('\r\n'))
		print(val, end="\n")
	except(UnicodeDecodeError):
		pass
