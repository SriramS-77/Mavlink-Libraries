from droneapi2 import Drone
import time

CONNECTION_STRING = 'tcp:127.0.0.1:5762'
VEHICLE_MODE = 'GUIDED'

drone = None

try:
    print(f'Connecting to drone on: {CONNECTION_STRING}')
    drone = Drone(CONNECTION_STRING)
    print('Connection established')

    print(f'Switching mode to {VEHICLE_MODE}')
    drone.change_mode(VEHICLE_MODE)
    print(f'Mode switch successful')

    print('Arming the drone...')
    drone.arm()
    print('Drone armed successfully')

    print("Waiting for 3 seconds...")
    time.sleep(3)

    drone.land()

except KeyboardInterrupt:
    print('Terminating script')

if drone is not None:
    print("Landing the drone")
    drone.land()
