from droneapi import Drone
import time
CONNECTION_STRING = 'tcp:127.0.0.1:5762'

TAKEOFF_ALTITUDE = 5
GROUND_SPEED = 5
AIR_SPEED = 10

# Waypoints
WAYPOINTS = [
    {'lat': -35.3627273, 'lon': 149.1650999, 'alt': 10},
    {'lat': -35.3631768, 'lon': 149.1650744, 'alt': 10},
    {'lat': -35.3633047, 'lon': 149.1649818, 'alt': 10},
    {'lat': -35.3634239, 'lon': 149.1650449, 'alt': 10}
]
drone = None

try:
    print(f'Connecting to drone on: {CONNECTION_STRING}')
    drone = Drone(CONNECTION_STRING)
    print('Connection established')

    drone.arm_and_takeoff(TAKEOFF_ALTITUDE, skip_prearm_checks=False, wait_armable=True)

    print('Starting mission...')
    drone.go_through_waypoints(WAYPOINTS, ground_speed=GROUND_SPEED, air_speed=AIR_SPEED)
    print('Mission completed successfully')

except KeyboardInterrupt as e:
    print("Terminating the script")

except Exception as e:
    print(f'Error occurred: {e}')
    print('Attempting emergency landing...')
    if drone is not None:
        drone.land()
        print('Drone landed safely')

if drone is not None:
    print("Landing the drone")
    drone.land()
