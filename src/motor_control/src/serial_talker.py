import serial
from time import sleep

COM = '/dev/ttyUSB1'
BAUD = 115200
MAXPWM = 1023 - 70;
MINPWM = 350;

def read_serial_data(ser):
    try:
        if ser.in_waiting > 0:  # if there is new data in the receive buffer, ie only trigger when u get new data
            
            # print(ser.in_waiting, end=' ')
            val = str(ser.readline().decode().strip('\r\n'))
            val = val.split()
            if (val[0] == "dx_vel" and len(val) == 5):
                # distance in micrometers, velocity in micrometer/milisec
                distance_right_wheel = val[1]
                distance_left_wheel = val[2]
                vel_right = val[3]
                vel_left = val[4]
            # print(dx, end="\n")
            print(val, end="\n")
        else:
            pass
            
    except(UnicodeDecodeError):
        pass

# def write_serial_data(ser):
#     # print("writing")
#     c = b"500,600\n"
#     # c = c.encode(encoding = 'utf-8')
#     ser.write(c)
#     ser.flush()
#     print(c)
#     sleep(1000)
#     # sleep(.001) # waiting for incomming data
#     # print("pass, is waiting")


with serial.Serial(COM, BAUD, timeout=1) as ser:
    print('Waiting for device')
    sleep(1)
    print("found {}".format(ser.name))
    ser.reset_output_buffer()
    ser.reset_input_buffer()

    while True:
        read_serial_data(ser)
        # write_serial_data(ser)
        
        
        # else:
        #     print("is not wainting")
