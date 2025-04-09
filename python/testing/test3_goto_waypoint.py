from droneapi2 import Drone
import time

# Connection details
CONNECTION_STRING = 'tcp:127.0.0.1:5762'
GROUND_SPEED = 2.0

# List of waypoints
WAYPOINTS = [
        {'lat': -35.3628891, 'lon': 149.1655049, 'alt': 10},
        {'lat': -35.3634885, 'lon': 149.1657731, 'alt': 15}
    ]
drone = None

try:
    print(f'Connecting to drone on: {CONNECTION_STRING}')
    drone = Drone(CONNECTION_STRING)
    print('Connection established')

    print('Taking off to initial altitude')
    drone.arm_and_takeoff(10)
    time.sleep(2)
    print('Starting waypoint navigation...')
    for i, waypoint in enumerate(WAYPOINTS):
        print(f'Moving to waypoint {i + 1}: {waypoint}')
        drone.goto_waypoint(waypoint['lat'], waypoint['lon'], waypoint['alt'])
        time.sleep(2)

    print('Successfully navigated through all waypoints')

    drone.goto_waypoint(drone.home_location['lat'], drone.home_location['lon'], 10)
    print("Reached back to home location!")

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
