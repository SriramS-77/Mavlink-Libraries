from pymavlink import mavutil, mavwp
from pymavlink.dialects.v20 import common as mavlink_common
import threading
import time
import math
import numpy as np

DEFAULT_MESSAGE_IDS = [mavutil.mavlink.MAVLINK_MSG_ID_AHRS,  # 163
                       mavutil.mavlink.MAVLINK_MSG_ID_AHRS2,  # 178
                       # mavutil.mavlink.MAVLINK_MSG_ID_AHRS3,  # 182
                       mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT,  # 24
                       mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD,  # 74
                       mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,  # 32,
                       mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,  # 33
                       mavutil.mavlink.MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT,  # 62
                       mavutil.mavlink.MAVLINK_MSG_ID_MISSION_ITEM_REACHED,
                       mavutil.mavlink.MAVLINK_MSG_ID_MISSION_CURRENT
                       ]

DEFAULT_MESSAGE_STREAMING_TIME_DURATION = 1  # Every 1sec ---> 1Hz

GUIDED = "GUIDED"
AUTO = "AUTO"
LAND = "LAND"
SMART_RTL = "SMART_RTL"
RTL = "RTL"

"""
Drone is the primary class, representing the vehicle to command. Uses a separate thread to receive message packets.
"""


class Drone:
    __conn: mavutil.mavtcp = None
    __target_system: int = None
    __target_component: int = None
    print_status: bool = True
    buffer: dict = {'HEARTBEAT': None,
                    'GLOBAL_POSITION_INT': None,
                    'NAV_CONTROLLER_OUTPUT': None,
                    'LOCAL_POSITION_NED': None,
                    'AHRS': None,
                    'AHRS2': None,
                    'AHRS3': None,
                    'VFR_HUD': None,
                    'MISSION_CURRENT': None,
                    'MISSION_ITEM_REACHED': None,
                    'COMMAND_ACK': None}

    def __init__(self, connection_string):
        """
        Drone is the primary class, representing the vehicle to command.
        :param connection_string: A string, representing the path or port through which the computer connects with and
                                  autonomously controls the vehicle.
        :return: nothing
        """
        self.__conn = mavutil.mavlink_connection(connection_string)
        self.__target_system = self.__conn.target_system
        self.__target_component = self.__conn.target_component
        print(f'Connection established to drone!')

        self.running = True
        self.listener_thread = threading.Thread(target=self.receive_message, daemon=True)
        self.listener_thread.start()
        print(f"Started listener thread!")

        self.command_ack_received = False
        self.command_result = None
        self.ack_lock = threading.Lock()

        self.configure_message_stream(message_ids=DEFAULT_MESSAGE_IDS,
                                      time_duration=DEFAULT_MESSAGE_STREAMING_TIME_DURATION)
        print(f"Configured message stream at {1 / DEFAULT_MESSAGE_STREAMING_TIME_DURATION}Hz")
        # print(self.wait_for_heartbeat())
        self.wait_for_heartbeat()
        print(f'Heartbeat received from system {self.__target_system}, component {self.__target_component}!')

        msg = self.get_global_location(blocking=True)
        self.home_lat, self.home_lon, self.home_alt = msg.lat / 1e7, msg.lon / 1e7, msg.alt / 1000
        self.home_location = {'lat': self.home_lat,
                              'lon': self.home_lon,
                              'alt': self.home_alt}

        print(f'\nHome location coordinates: {self.home_location}\n')

    def connection(self):
        return self.__conn

    def close(self):
        self.__conn.close()

    def receive_message(self):
        while self.running:
            msg = self.__conn.recv_match(type=list(self.buffer.keys()), blocking=True)
            if msg is None:
                return
            if msg.get_type() == "COMMAND_ACK":
                self.handle_command_ack(msg)
            if msg.get_type() in self.buffer.keys():
                self.buffer[msg.get_type()] = msg  # Store in queue for later processing
                # print(msg)
            time.sleep(0.1)

    def update_command_ack(self, received, result=None):
        with self.ack_lock:
            self.command_ack_received = received
            self.command_result = result

    def clear_command_ack(self):
        self.update_command_ack(received=False, result=None)

    def handle_command_ack(self, msg):
        self.update_command_ack(received=True, result=msg.result)

    def clear_mission_data(self):
        self.buffer["MISSION_ITEM_REACHED"] = None
        self.buffer["MISSION_CURRENT"] = None

    def wait_for_command_ack(self, timeout=5):
        start_time = time.time()
        while timeout is None or time.time() - start_time < timeout:
            with self.ack_lock:
                if self.command_ack_received:
                    return self.command_result == mavlink_common.MAV_RESULT_ACCEPTED, self.command_result
            time.sleep(0.1)
        print("Timeout waiting for command acknowledgment.")
        return False, None

    def wait_for_heartbeat(self, timeout=5):
        while self.buffer['HEARTBEAT'] is None:
            time.sleep(1)
        print(self.buffer['HEARTBEAT'])
        return

    def __send_mavlink(self, msg):
        self.__conn.mav.send(msg)

    def __send_command(self, msg):
        self.clear_command_ack()
        self.__send_mavlink(msg)

    def __long_encode(self,
                      command: int,
                      confirmation: int = 0,
                      param1: float = 0,
                      param2: float = 0,
                      param3: float = 0,
                      param4: float = 0,
                      param5: float = 0,
                      param6: float = 0,
                      param7: float = 0):
        return self.__conn.mav.command_long_encode(self.__target_system, self.__target_component,
                                                   command, confirmation,
                                                   param1, param2, param3, param4, param5, param6, param7)

    def confirmation_message(self, task: str, timeout=5, blocking=True):
        """
        Mostly used internally. Confirms the successful completion of a command, or gives the state of the command or reason for failure.

        :param task:    Type (str). It specifies the name of the task/command.
        :param blocking: Type (bool). Decides whether to wait indefinitely for the acknowledgement message.
        :param timeout: Type (int). It is the time in seconds to wait for the acknowledgement message, given blocking = False.
        :return: nothing
        """
        confirmation, result = self.wait_for_command_ack(timeout=timeout)
        if result is None:
            print(f"{task} Timeout!")
        elif result == 0:
            print(f"{task} Successful!")
        elif result == 4:
            print(f"{task} Unsuccessful!")
        elif result == 3:
            print(f"{task} - This command is not supported.")
        elif result == 2:
            print(f"{task} - Invalid command. -> Command is supported, but invalid parameters.")
        elif result == 1:
            print(f"{task} - Temporarily rejected. -> Problem will be fixed by waiting, try again.")
        elif result == 5:
            print(f"{task} - In progress...")
        elif result == 9:
            print(f"{task} - Failed. -> Given mav_frame is not supported.")
        elif result == 6:
            print(f"{task} - Command was cancelled.")
        else:
            print(f"{task} - Command failed. -> Result = {result}")
        return

    def __stream_msg(self, msg_id: int, time_duration: float = 1):
        msg = self.__long_encode(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,  # ID of command to send
                                 0,  # Confirmation
                                 msg_id,  # MAVLINK_MSG_ID_BATTERY_STATUS  # Message ID to be streamed
                                 time_duration * 1e6,  # param2: Interval in microseconds
                                 0, 0, 0, 0, 0
                                 )
        self.__send_mavlink(msg)

    def configure_message_stream(self, message_ids, time_duration: float = 1):
        for message_id in message_ids:
            self.__stream_msg(msg_id=message_id, time_duration=time_duration)

    def pre_arm_checks(self):
        """
        Executes pre-arm safety checks.

        :return: nothing
        """
        print(f'Doing pre-arm checks...')
        msg = self.__long_encode(mavutil.mavlink.MAV_CMD_RUN_PREARM_CHECKS, 0, 0, 0, 0, 0, 0, 0, 0)
        self.__send_mavlink(msg)
        # Confirmation Message
        result = self.confirmation_message(task="Pre-arm checks")
        checks_passed = True if result == 0 else False
        return checks_passed

    def arm(self, skip_prearm_checks: bool = False, wait_armable: bool = True):
        """
        Arms the vehicle. Might perform pre-arm safety checks, depending on the command.

        :param skip_prearm_checks: This parameter controls whether to perform the pre-arm safety checks or to skip them.
        :param wait_armable: This parameter controls whether to wait till the drone is armable, by passing prearm checks.
        :return: nothing
        """
        if not skip_prearm_checks:
            while True:
                is_armable = self.pre_arm_checks()
                if is_armable:
                    print("Pre-arm checks passed. Drone is armable.")
                    break
                else:
                    print("Drone is not armable. Waiting for 3 sec...")
                    time.sleep(3)
                if not wait_armable:
                    break

        print(f'Arming...')

        msg = self.__long_encode(command=mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                 param1=1)
        self.__send_command(msg)

        self.confirmation_message(task="Arming", timeout=5)
        # print("Armed!")

    def disarm(self, force_disarm: bool = False):
        """
        Disarms the vehicle.

        :param force_disarm: Boolean argument which decides whether to forcefully disarm the drone, even when it is in the air.
        :return: nothing
        """
        disarm_param = 21196 if force_disarm else 0
        print(f'Disarming...')
        msg = self.__long_encode(command=mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                 param2=disarm_param)
        self.__send_mavlink(msg)

        self.confirmation_message(task="Disarm", timeout=3)
        # print("Disarmed!")

    def is_armed(self) -> bool:
        heartbeat = self.buffer.get("HEARTBEAT", None)
        if heartbeat is not None:
            base_mode = heartbeat.base_mode   # Get the base_mode from the heartbeat message
            armed = base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED   # Check if the vehicle is armed
            return armed

    def change_mode(self, mode: str = 'GUIDED'):
        """
        Autonomously changes the mode of the vehicle.

        :param mode: Type (str). Default -> 'GUIDED'.
                     It is the new mode for the vehicle.  ['GUIDED', 'AUTO', 'RTL', 'LAND', 'LOITER', etc.]
        :return: nothing
        """
        mode = mode.upper().strip()
        # Gets the possible modes of the vehicle and their id's, as a dictionary.
        mode_id = self.__conn.mode_mapping()[mode]
        # self.__conn.set_mode(mode_id)
        msg = self.__long_encode(command=mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                                 param1=mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                                 param2=mode_id)
        self.__send_command(msg=msg)

        self.confirmation_message(f'Setting Mode to {mode}', timeout=5, blocking=False)   # Confirmation Message

    def get_mode(self):
        return   # self.mode

    def arm_and_takeoff(self,
                        takeoff_altitude,
                        verbose=True,
                        skip_prearm_checks=False,
                        wait_armable=True):
        """
        Arms the vehicle and takes off to the desired altitude, relative to the home location.

        :param takeoff_altitude:   This is in metres, relative to the home location.
        :param verbose:            Controls display of altitude update messages.
        :param skip_prearm_checks: This parameter controls whether to perform the pre-arm safety checks or to skip them.
        :param wait_armable:       This parameter controls whether to wait till drone is arm-able.
        :return: nothing
        """
        # Changing mode to 'GUIDED'
        self.change_mode(mode="GUIDED")

        # Arming
        self.arm(skip_prearm_checks=skip_prearm_checks, wait_armable=wait_armable)

        if takeoff_altitude is None or takeoff_altitude == 0:
            print("Invalid takeoff altitude!")
            return
        # Takeoff
        print(f'Taking off to altitude of {takeoff_altitude} metres...')
        msg = self.__long_encode(command=mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                                 param7=takeoff_altitude)
        self.__send_command(msg)

        self.confirmation_message(task=f"Takeoff to {takeoff_altitude}m", timeout=5)

        # Altitude Checking
        while True:
            msg = self.get_global_location()
            if verbose:
                print(f"Relative Altitude: {msg.relative_alt / 1000}")
            if msg.relative_alt > takeoff_altitude * 1000 * 0.98:
                break
            time.sleep(1)
        # Target reached message
        print(f"Reached target altitude of {takeoff_altitude} metres!")

    def land(self, wait_till_landed: bool = True, verbose: bool = True):
        """
        Lands the vehicle on the ground. Might disarm it, depending on the command given.

        :param wait_till_landed: Wait till landed before exiting function
        :param verbose: Controls display of altitude update messages
        :return: nothing
        """
        # Landing
        print(f'Landing...')
        msg = self.__long_encode(command=mavutil.mavlink.MAV_CMD_NAV_LAND)
        self.__send_command(msg)

        # Confirmation Message
        self.confirmation_message(task="Landing", timeout=3)

        if wait_till_landed:
            while self.is_armed():
                if verbose:
                    print("Vehicle is landing, waiting for 3 sec...")
                time.sleep(3)

        print(f'Vehicle Landed and Disarmed!')

    def goto_waypoint(self, target_lat, target_lon, target_alt=None, relative_frame: bool = True, verbose: bool = True):
        """
        Goes to a waypoint, based on the global latitude and longitude coordinates. The target altitude is relative to the home location.

        :param target_lat:     Type (float). It is the unscaled latitude value of the target waypoint.
        :param target_lon:     Type (float). It is the unscaled longitude value of the target waypoint.
        :param target_alt:     Type (int). It is the target altitude, relative to the home location, in metres.
        :param relative_frame: Type (bool). Controls frame of reference (Home altitude / Mean sea level).
        :param verbose:        Controls display of location update messages.
        :return: nothing
        """
        if target_alt is None:
            loc = self.get_location(blocking=True)
            target_alt = loc['alt']
            relative_frame = True

        frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT if relative_frame else mavutil.mavlink.MAV_FRAME_GLOBAL_ALT_INT

        target_lat = int(target_lat * 1e7)
        target_lon = int(target_lon * 1e7)
        # Command for location change
        msg = mavutil.mavlink.MAVLink_set_position_target_global_int_message(0,
                                                                             self.__target_system,
                                                                             self.__target_component,
                                                                             frame,
                                                                             int(0b110111111000),
                                                                             target_lat, target_lon, target_alt,
                                                                             0, 0, 0, 0, 0, 0, 0, 0)
        self.__send_mavlink(msg)

        # Location checking
        print('Heading to waypoint!')
        start = False
        while True:
            msg = self.get_nav_controller()
            if msg is None:
                time.sleep(3)
                continue
            if verbose:
                print(f'Distance to waypoint: {msg.wp_dist}')
            if msg.wp_dist > 0:
                start = True
            elif msg.wp_dist == 0 and start:
                break
            time.sleep(3)

        print("\nWaypoint reached!!!\n")
        return

    def change_altitude_to(self, target_alt, relative_frame: bool = True, verbose: bool = False):
        """
        Changes altitude of the vehicle. Target altitude is given with respect to the home location.

        :param target_alt: Type (int). It is the target altitude, relative to the home location, in metres.
        :param relative_frame: Type(bool). Controls frame of reference (Home altitude / Mean sea level).
        :param verbose:    Controls display of altitude update messages.
        :return: nothing
        """
        # Command for altitude change
        if relative_frame:
            print(f'Changing altitude to {target_alt} metres above home location')
        else:
            print(f'Changing altitude to {target_alt} metres above mean sea level')

        frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT if relative_frame else mavutil.mavlink.MAV_FRAME_GLOBAL_ALT_INT

        loc = self.get_location(blocking=True)
        target_lat = int(loc['lat'] * 1e7)
        target_lon = int(loc['lon'] * 1e7)

        # Command for location change
        msg = mavutil.mavlink.MAVLink_set_position_target_global_int_message(0, self.__target_system,
                                                                             self.__target_component,
                                                                             frame,
                                                                             int(0b110111111000),
                                                                             target_lat, target_lon, target_alt,
                                                                             0, 0, 0, 0, 0, 0, 0, 0)
        self.__send_mavlink(msg)

        # Altitude checking
        while True:
            loc = self.get_location(relative_frame=relative_frame)
            alt_diff = target_alt - loc['alt']
            if verbose:
                print(f'Distance to target altitude: {alt_diff} metres')
            if abs(alt_diff) <= 0.5:
                break
            time.sleep(3)
        # Confirmation Message
        print(f"\nAltitude of {target_alt} metres reached!\n")

    def goto_ned_position(self, north_displacement, east_displacement, up_displacement, verbose=True):
        """
        Changes the position of the vehicle, based on North, East and Up directions. Units are in metres.

        :param north_displacement: Type (int). Distance to move in North direction in metres.
        :param east_displacement:  Type (int). Distance to move in East direction in metres.
        :param up_displacement:    Type (int). Distance to move up in metres. [Up -> Positive, Down -> Negative]
        :param verbose:            Controls display of location update messages.
        :return: nothing
        """
        # Command for location change
        print(f"Moving {north_displacement} metres North,"
              f"{east_displacement} metres East,"
              f"{up_displacement} metres UP...")
        self.__conn.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(0, self.__target_system,
                                                                                           self.__target_component,
                                                                                           mavutil.mavlink.MAV_FRAME_BODY_NED,
                                                                                           int(0b110111111000),
                                                                                           north_displacement,
                                                                                           east_displacement,
                                                                                           up_displacement * -1,
                                                                                           0, 0, 0, 0, 0, 0, 0, 0))
        # Location checking
        start = False
        while True:
            msg = self.get_nav_controller()   # self.__conn.recv_match(type='NAV_CONTROLLER_OUTPUT', blocking=True)
            if msg is None:
                time.sleep(3)
                continue
            if verbose:
                print(f'Distance to target altitude: {msg.wp_dist} metres')
            if msg.wp_dist > 0:
                start = True
            elif msg.wp_dist == 0 and start:
                break
            time.sleep(3)
        # Confirmation Message
        print(f'New position reached!')
        print(f"Moved {north_displacement} metres North, {east_displacement} metres East, {up_displacement} metres UP!")

    def change_altitude_by(self, difference_altitude: float, verbose: bool = False):
        """
        Changes the altitude by difference_altitude. Up is positive, Down is negative. Units are in metres.

        :param difference_altitude: Type (float). Distance to move up in metres.
        :param verbose: Flag for printing messages.
        :return: nothing
        """
        print(f'Changing altitude by {difference_altitude} metres...')

        cur_loc = self.get_location(relative_frame=True)
        target_alt = cur_loc['alt'] + difference_altitude

        dist_down = difference_altitude * -1
        # Command for altitude change
        msg = mavutil.mavlink.MAVLink_set_position_target_local_ned_message(0, self.__target_system,
                                                                            self.__target_component,
                                                                            mavutil.mavlink.MAV_FRAME_BODY_NED,
                                                                            int(0b110111111000),
                                                                            0, 0, dist_down,
                                                                            0, 0, 0, 0, 0, 0, 0, 0)
        self.__send_mavlink(msg)

        # Altitude checking
        while True:
            loc = self.get_location(relative_frame=True)
            alt_diff = target_alt - loc['alt']
            if verbose:
                print(f'Distance to target altitude: {alt_diff} metres')
            if abs(alt_diff) <= 0.5:
                break
            time.sleep(3)
        # Confirmation Message
        print(f'Moved up {difference_altitude} metres!') if difference_altitude > 0 else print(
            f'Moved down {difference_altitude} metres!')

    def move_ned(self, dist_north: float, dist_east: float, dist_down: float = 0, max_axis_speed: float = 1):
        """
        Moves the vehicle using velocity decomposition, based on North, East and Down directions. Units are in metres.

        :param dist_north: Type (int). Distance to move in North direction in metres.
        :param dist_east: Type (int). Distance to move in East direction in metres.
        :param dist_down: Type (int). Distance to move down in metres. [Down -> Positive, Up -> Negative]
        :param max_axis_speed: Maximum allowed speed in any NED axis, in m/s.
        :return: nothing
        """
        duration_north, duration_east, duration_down = abs(dist_north / max_axis_speed), abs(
            dist_east / max_axis_speed), abs(dist_down / max_axis_speed)
        duration = max(duration_north, duration_east, duration_down)
        duration = math.ceil(duration)
        velocity_north, velocity_east, velocity_down = dist_north / duration, dist_east / duration, dist_down / duration

        self.__send_global_velocity(velocity_north, velocity_east, velocity_down, duration=duration)
        self.__send_global_velocity(0, 0, 0, 1)

    def __send_global_velocity(self, velocity_north, velocity_east, velocity_down, duration):
        duration = int(duration)
        msg = mavutil.mavlink.MAVLink_set_position_target_local_ned_message(0,
                                                                            self.__target_system,
                                                                            self.__target_component,
                                                                            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                                                                            int(0b111111000111),
                                                                            0, 0, 0,  # Distances
                                                                            velocity_north,
                                                                            velocity_east,
                                                                            velocity_down,
                                                                            0, 0, 0,  # Accelerations
                                                                            0, 0  # Yaw, Yaw-rate
                                                                            )

        # Send command to vehicle on 1 Hz cycle
        for x in range(0, duration):
            # Command for velocity change
            self.__send_mavlink(msg)
            time.sleep(1)

    @staticmethod
    def decompose_distance(distance, heading, axis):
        heading = (heading + 90) % 360 if axis == 'X' else heading
        heading = np.deg2rad(heading)
        dist_north = distance * np.cos(heading)
        dist_east = distance * np.sin(heading)
        return dist_north, dist_east

    def move_3d(self, dist_x: float, dist_y: float, dist_z: float = 0, max_axis_speed: float = 1):
        heading = self.get_heading()

        dist_x_north, dist_x_east = self.decompose_distance(dist_x, heading, 'X')
        dist_y_north, dist_y_east = self.decompose_distance(dist_y, heading, 'Y')

        dist_north = dist_x_north + dist_y_north
        dist_east = dist_x_east + dist_y_east
        dist_down = dist_z * -1

        duration_north, duration_east, duration_down = abs(dist_north / max_axis_speed), abs(
            dist_east / max_axis_speed), abs(dist_down / max_axis_speed)
        duration = max(duration_north, duration_east, duration_down)
        duration = max(1, math.ceil(duration))
        velocity_north, velocity_east, velocity_down = dist_north / duration, dist_east / duration, dist_down / duration

        self.__send_global_velocity(velocity_north, velocity_east, velocity_down, duration=duration)
        self.__send_global_velocity(0, 0, 0, 2)

    def create_mission(self, waypoints):
        wp = mavwp.MAVWPLoader()
        cur_loc = self.get_location(relative_frame=True)
        mission_cmd = self.__conn.mav.mission_item_encode(self.__target_system, self.__target_component,
                                                          0,  # seq
                                                          mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,  # frame
                                                          mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,  # Command
                                                          0,  # Current
                                                          1,  # Auto-continue
                                                          0, 0, 0, math.nan,
                                                          cur_loc['lat'], cur_loc['lon'], cur_loc['alt']
                                                          )
        wp.add(mission_cmd)

        for mission_idx, waypoint in enumerate(waypoints):
            lat = waypoint['lat']
            lon = waypoint['lon']
            alt = waypoint['alt']

            mission_cmd = self.__conn.mav.mission_item_encode(self.__target_system, self.__target_component,
                                                              mission_idx + 1,  # seq
                                                              mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,  # frame
                                                              mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,  # Command
                                                              0,  # Current
                                                              1,  # Auto-continue
                                                              0, 0, 0, math.nan,
                                                              lat, lon, alt
                                                              )
            wp.add(mission_cmd)
        # Adding last waypoint once more to initiate waiting
        # wp.add(mission_cmd)

        return wp

    def send_mission(self, wp: mavwp.MAVWPLoader):
        self.clear_mission_data()
        self.__conn.waypoint_clear_all_send()
        self.__conn.waypoint_count_send(wp.count())

        for _ in range(wp.count()):
            msg = self.__conn.recv_match(type=['MISSION_REQUEST'], blocking=True)
            print(msg)
            self.__conn.mav.send(wp.wp(msg.seq))
            print(f'Sending waypoint {msg.seq}')

        # msg = self.__conn.mav.mission_set_current_encode(self.__target_system, self.__target_component, 0)
        # self.__send_mavlink(msg)
        msg = self.__conn.recv_match(type='MISSION_ACK', blocking=True)
        print("Mission acknowledged!")
        print(msg)

    def go_through_waypoints(self, waypoints, ground_speed, air_speed):
        wp = self.create_mission(waypoints=waypoints)
        waypoints_count = wp.count()

        self.send_mission(wp=wp)

        self.set_speed(target_ground_speed=ground_speed, target_air_speed=air_speed)

        print("Changing mode to AUTO...")
        self.change_mode('AUTO')

        while True:
            time.sleep(3)
            msg = self.buffer["MISSION_ITEM_REACHED"]
            print(msg) if msg else None

            msg = self.buffer["MISSION_CURRENT"]
            if msg is None:
                continue
            print(msg)

            print(f"Distance to next waypoint: {self.get_wpdist()}m")

            if msg.seq == waypoints_count - 1 and msg.mission_state == 5:
                print("Mission completed successfully!")
                time.sleep(3)
                break

        time.sleep(1)

        print("Changing mode back to GUIDED...")
        self.change_mode('GUIDED')

        return

    def __set_ground_speed(self, ground_speed):
        msg = self.__long_encode(mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
                                 0,
                                 1,  # Speed type
                                 ground_speed,  # Ground speed
                                 -1,  # Indicates no change in throttle
                                 0, 0, 0, 0)
        self.__send_mavlink(msg)
        # Confirmation Message
        msg = self.__conn.recv_match(type="COMMAND_ACK", blocking=True)
        print(msg)
        return

    def __set_air_speed(self, air_speed):
        msg = self.__long_encode(mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
                                 0,
                                 0,  # Speed type
                                 air_speed,  # Air speed
                                 -1,  # Indicates no change in throttle
                                 0, 0, 0, 0)
        self.__send_mavlink(msg)
        # Confirmation Message
        msg = self.__conn.recv_match(type="COMMAND_ACK", blocking=True)
        print(msg)
        return

    def set_speed(self, target_ground_speed: float = None, target_air_speed: float = None):
        """
        Changes the ground and air speed of the vehicle.

        :param target_ground_speed: Type (float). The target ground speed for the vehicle, in metres per second (m/s).
        :param target_air_speed: Type (float). The target air speed for the vehicle, in metres per second (m/s).
        :return: nothing
        """
        # Command for ground speed change
        if target_ground_speed is not None:
            self.__set_ground_speed(ground_speed=target_ground_speed)
            print(f'Ground-speed of drone was changed to {target_ground_speed}\n')

        # Command for ground speed change
        if target_air_speed is not None:
            self.__set_air_speed(air_speed=target_air_speed)
            print(f'Air-speed of drone was changed to {target_air_speed}\n')

        return

    def get_nav_controller(self):
        return self.buffer.get("NAV_CONTROLLER_OUTPUT", None)

    def get_wpdist(self):
        nav_status = self.get_nav_controller()
        return nav_status.wp_dist

    def get_global_location(self, blocking: bool = False):
        msg = self.buffer.get("GLOBAL_POSITION_INT", None)
        while msg is None and blocking is True:
            msg = self.buffer.get("GLOBAL_POSITION_INT", None)
            time.sleep(0.5)
        return msg

    def get_location(self, relative_frame: bool = True, blocking: bool = False):
        msg = self.get_global_location(blocking=blocking)
        if msg is None:
            return None
        lat, lon, alt = msg.lat / 1e7, msg.lon / 1e7, msg.alt / 1000
        if relative_frame:
            alt = alt - self.home_location['alt']
        current_location = {"lat": lat,
                            "lon": lon,
                            "alt": alt}
        return current_location

    def show_attitude(self):
        print(f'Attitude of vehicle:')
        print(self.buffer["AHRS"])
        print(self.buffer["AHRS2"])
        print(self.buffer["VFR_HUD"])
        # print(self.__conn.recv_match(type="AHRS3", blocking=True))
        print()

    def get_heading(self):
        """
        msg = self.__conn.recv_match(type="AHRS2", blocking=True)

        yaw = msg.yaw
        pitch = msg.pitch
        roll = msg.roll
        # print(msg)
        yaw_degrees = yaw * 180 / math.pi
        yaw_degrees = yaw_degrees % 360
        print(f"Yaw in degrees: {yaw_degrees}...")
        """
        msg = self.get_global_location()
        yaw_degrees = msg.hdg / 100
        print(f"Yaw from global position: {yaw_degrees}...")
        return yaw_degrees

    def show_speed(self):
        print(f'Speed of vehicle:')
        msg = self.buffer["VFR_HUD"]
        print(msg, '\n')

    def get_ground_speed(self):
        msg = self.buffer["VFR_HUD"]   # self.__conn.recv_match(type="VFR_HUD", blocking=True)
        ground_speed = msg.groundspeed if msg is not None else None
        return ground_speed

    def get_air_speed(self):
        msg = self.buffer["VFR_HUD"]   # self.__conn.recv_match(type="VFR_HUD", blocking=True)
        air_speed = msg.airspeed if msg is not None else None
        return air_speed
