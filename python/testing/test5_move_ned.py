from droneapi2 import Drone
import time

# Connection details
CONNECTION_STRING = 'tcp:127.0.0.1:5762'


TAKEOFF_ALTITUDE = 5

MOVEMENTS = [
    {'north': 5, 'east': 5, 'down': 0},
    {'north': -10, 'east': 10, 'down': 0},
    {'north': 0, 'east': 0, 'down': -5},
    {'north': 5, 'east': 15, 'down': 3},
]

drone = None

try:
    print(f'Connecting to drone on: {CONNECTION_STRING}')
    drone = Drone(CONNECTION_STRING)
    print('Connection established')

    drone.arm_and_takeoff(TAKEOFF_ALTITUDE)

    print('Starting NED movement...')
    for i, move in enumerate(MOVEMENTS):
        print(f'Moving: {move}')
        drone.move_ned(move['north'], move['east'], move['down'])

        print("Hovering for 3sec...")
        time.sleep(3)

    print('Successfully completed NED movement')

except KeyboardInterrupt as e:
    print("Terminating script")

except Exception as e:
    print(f'Error occurred: {e}')
    print('Attempting emergency landing...')
    if drone is not None:
        drone.land()
        print('Drone landed safely')

if drone is not None:
    print("Landing the drone")
    drone.land()
