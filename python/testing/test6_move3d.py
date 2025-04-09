from droneapi2 import Drone
import time

# Connection details
CONNECTION_STRING = 'tcp:127.0.0.1:5762'


TAKEOFF_ALTITUDE = 5

MOVEMENTS = [
    {'x': 5, 'y': 5, 'z': 5},
    {'x': -3, 'y': 0, 'z': 0},  
    {'x': 0, 'y': -5, 'z': 2}
]

drone = None

try:
    print(f'Connecting to drone on: {CONNECTION_STRING}')
    drone = Drone(CONNECTION_STRING)
    print('Connection established')

    drone.arm_and_takeoff(TAKEOFF_ALTITUDE)

    print('Starting 3D movement...')
    for i, move in enumerate(MOVEMENTS):
        print(f'Moving: {move}')
        drone.move_3d(move['x'], move['y'], move['z'])
        time.sleep(2)

    print('Successfully completed 3D movement')

    drone.land()

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
