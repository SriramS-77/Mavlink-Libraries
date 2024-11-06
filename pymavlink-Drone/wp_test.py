from drone_class import Drone

connection_string = 'tcp:127.0.0.1:5762'    # 'udp:127.0.0.1:14550'

master = Drone(connection_string=connection_string)

try:
    master.get_attitude()
    master.get_speed()

    master.arm_and_takeoff(5, skip_prearm_checks=False, wait_armable=True)

    master.set_speed(target_ground_speed=15)

    # master.goto_waypoint(-35.3645384, 149.1648531, 30)

    # master.set_speed(target_ground_speed=10)

    # master.goto_waypoint(-35.3633572, 149.1652072, 5)

    # master.change_altitude_to(25, verbose=True)

    master.goto_ned_position(20, -20, 5)

    master.change_altitude_by(10, verbose=True)

    master.land()

except KeyboardInterrupt as e:
    print(f'KeyBoard Interrupt Detected ---> Emergency Landing Initiated!')
    master.land()
