from pymavlink import mavutil

conn = mavutil.mavlink_connection('/dev/ttyACM0', baud=57600)

while True:
    msg = conn.recv_match(blocking=True)
    if msg:
        print(msg)