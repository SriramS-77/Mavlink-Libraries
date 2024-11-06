from drone_class import Drone
import time

connection_string = 'tcp:127.0.0.1:5762'    # 'udp:127.0.0.1:14550'

master = Drone(connection_string=connection_string)

try:
    loc = master.connection().location()
    print(loc)

    master.get_attitude()
    master.get_speed()

    master.get_heading()

    master.arm_and_takeoff(5, skip_prearm_checks=True)

    master.move_ned(-5, 0)
    master.get_speed()
    master.get_heading()
    time.sleep(5)

    master.move_ned(-5, -5)
    master.get_speed()
    master.get_heading()
    time.sleep(5)

    master.move_ned(10, -20)
    master.get_speed()
    master.get_heading()
    time.sleep(5)

    master.get_heading()
    master.get_speed()
    master.get_heading()
    time.sleep(5)

    master.land()

except KeyboardInterrupt as e:
    print(f'KeyBoard Interrupt Detected ---> Emergency Landing Initiated!')
    master.land()
