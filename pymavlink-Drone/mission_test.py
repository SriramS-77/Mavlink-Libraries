from drone_class import Drone
import time

connection_string = 'tcp:127.0.0.1:5762'

master = Drone(connection_string=connection_string)

mission_lst = [{'latitude': -35.3628891, 'longitude': 149.1655049, 'altitude': 10},
               {'latitude': -35.3634885, 'longitude': 149.1657731, 'altitude': 15},
               {'latitude': -35.3635497, 'longitude': 149.1648933, 'altitude': 20},
               {'latitude': -35.3630204, 'longitude': 149.1649175, 'altitude': 5}]

try:
    master.get_attitude()
    master.get_speed()

    master.arm_and_takeoff(5, skip_prearm_checks=False, wait_armable=True)

    time.sleep(10)

    master.mission(waypoints=mission_lst)

    master.land()

except KeyboardInterrupt as e:
    print(f'KeyBoard Interrupt Detected ---> Emergency Landing Initiated!')
    master.land()
