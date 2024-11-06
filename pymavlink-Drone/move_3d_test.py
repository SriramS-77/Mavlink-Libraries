from drone_class import Drone
import time

connection_string = 'tcp:127.0.0.1:5762'   #'udp:127.0.0.1:14552'

master = Drone(connection_string=connection_string)

try:
    master.change_mode()
    loc = master.connection().location()
    print(loc)

    master.get_attitude()
    master.get_speed()

    master.get_heading()

    master.arm_and_takeoff(5, skip_prearm_checks=True)

    while True:
        x, y = input("Enter x and y distances to move: ").split()
        if x == 'x' and y == 'x':
            break
        x = int(x)
        y = int(y)
        master.move_3d(x, y, max_axis_speed=5)
        try:
            # master.change_mode('auto')
            counter = 0
            while True:
                counter += 1
                print(f"\nCounter: {counter}")
                master.get_attitude()
                master.get_heading()
                time.sleep(1)
        except KeyboardInterrupt:
            # master.change_mode()
            continue

    master.land()

except KeyboardInterrupt as e:
    print(f'KeyBoard Interrupt Detected ---> Emergency Landing Initiated!')
    master.land()
