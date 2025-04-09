from droneapi2 import Drone
import time

# Connection details
CONNECTION_STRING = 'tcp:127.0.0.1:5762'
TAKEOFF_ALTITUDE = 10  # Meters

drone = None

try:
    print(f'Connecting to drone on: {CONNECTION_STRING}')
    drone = Drone(CONNECTION_STRING)
    print('Connection established')

    print(f'Arming and taking off to {TAKEOFF_ALTITUDE} meters...')
    drone.arm_and_takeoff(TAKEOFF_ALTITUDE)
    print('Takeoff successful')

    print("Hovering for 3 seconds...")
    time.sleep(3)

    drone.land()

except KeyboardInterrupt as e:
    print('Terminating Script')

except Exception as e:
    print(f'Error occurred: {e}')
    print('Attempting emergency landing...')
    if drone is not None:
        drone.land()
        print('Drone landed safely')

if drone is not None:
    print("Landing the drone")
    drone.land()
