from pymavlink import mavutil, mavwp
import time
import math
import numpy as np


DEFAULT_MESSAGE_IDS = [mavutil.mavlink.MAVLINK_MSG_ID_AHRS,   # 163
                       mavutil.mavlink.MAVLINK_MSG_ID_AHRS2,  # 178
                       # mavutil.mavlink.MAVLINK_MSG_ID_AHRS3,  # 182
                       mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT,   # 24
                       mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD,   # 74
                       mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,   # 32,
                       mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,   # 33
                       mavutil.mavlink.MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT   # 62
                       ]

DEFAULT_MESSAGE_STREAMING_TIME_DURATION = 1   # Every 1sec ---> 1Hz

GUIDED = "GUIDED"
AUTO = "AUTO"
LAND = "LAND"
SMART_RTL = "SMART_RTL"
RTL = "RTL"

"""
Drone is the primary class, representing the vehicle to command.
"""


class Drone:
    def __init__(self, connection_string):
        """
        Drone is the primary class, representing the vehicle to command.
        :param connection_string: A string, representing the path or port through which the computer connects with and
                                  autonomously controls the vehicle.
        :return: nothing
        """
        self.__conn = mavutil.mavlink_connection(connection_string)
        self.__conn.wait_heartbeat()
        print(f'Connection established to drone!')
        print(f'Heartbeat received from system {self.__conn.target_system}, component {self.__conn.target_component}!')

        self.__target_system = self.__conn.target_system
        self.__target_component = self.__conn.target_component

        self.configure_message_stream(message_ids=DEFAULT_MESSAGE_IDS,
                                      time_duration=DEFAULT_MESSAGE_STREAMING_TIME_DURATION)

        msg = self.__conn.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        self.lat, self.lon, self.alt = msg.lat / 1e7, msg.lon / 1e7, msg.alt / 1000
        print(f'\nHome location coordinates: {self.lat}, {self.lon}, {self.alt}\n')
        self.home_location = {"lat": self.lat,
                              "lon": self.lon,
                              "alt": self.alt}

    def connection(self):
        return self.__conn

    def __send_mavlink(self, msg):
        self.__conn.mav.send(msg)

    def __long_encode(self, command: int, confirmation: int, param1: float, param2: float, param3: float, param4: float, param5: float, param6: float, param7: float):
        return self.__conn.mav.command_long_encode(self.__target_system, self.__target_component,
                                                   command, confirmation,
                                                   param1, param2, param3, param4, param5, param6, param7)

    def confirmation_message(self, task: str, blocking=True, timeout=None):
        """
        Mostly used internally. Confirms the successful completion of a command, or gives the state of the command or reason for failure.

        :param task:    Type (str). It specifies the name of the task/command.
        :param blocking: Type (bool). Decides whether to wait indefinitely for the acknowledgement message.
        :param timeout: Type (int). It is the time in seconds to wait for the acknowledgement message, given blocking = False.
        :return: nothing
        """
        msg = self.__conn.recv_match(type="COMMAND_ACK", blocking=blocking, timeout=timeout)
        if msg.result == 0:
            print(f"{task} Successful!")
        elif msg.result == 4:
            print(f"{task} Unsuccessful!")
        elif msg.result == 3:
            print(f"{task} - This command is not supported.")
        elif msg.result == 2:
            print(f"{task} - Invalid command. -> Command is supported, but invalid parameters.")
        elif msg.result == 1:
            print(f"{task} - Temporarily rejected. -> Problem will be fixed by waiting, try again.")
        elif msg.result == 5:
            print(f"{task} - In progress...")
        elif msg.result == 9:
            print(f"{task} - Failed. -> Given mav_frame is not supported.")
        elif msg.result == 6:
            print(f"{task} - Command was cancelled.")
        else:
            print(f"{task} - Command failed. -> Result = {msg.result}")

        return msg

    def __stream_msg(self, msg_id: int, time_duration: float = 1):
        msg = self.__long_encode(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,  # ID of command to send
                                 0,  # Confirmation
                                 msg_id,  # MAVLINK_MSG_ID_BATTERY_STATUS  # Message ID to be streamed
                                 time_duration * 1e6,  # param2: Interval in microseconds
                                 0,  0,  0,  0,  0
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
        msg = self.confirmation_message(task="Pre-arm checks", blocking=True)
        print("Pre-arm checks message:", msg)
        checks_passed = True if msg.result == 0 else False
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

        self.__conn.mav.command_long_send(
            self.__target_system,
            self.__target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0)
        # Confirmation Message
        msg = self.__conn.recv_match(type="COMMAND_ACK", blocking=True)
        print(msg)
        print('Armed!')
    
    def disarm(self, force_disarm: bool = False):
        """
        Disarms the vehicle.

        :param force_disarm: Boolean argument which decides whether to forcefully disarm the drone, even when it is in the air.
        :return: nothing
        """
        disarm_param = 21196 if force_disarm else 0
        print(f'Disarming...')
        msg = self.__long_encode(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, disarm_param, 0, 0, 0, 0, 0)
        self.__send_mavlink(msg)
        # Confirmation Message
        self.confirmation_message(task="Disarm", blocking=True)
        print("Disarmed!")

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
        self.__conn.mav.command_long_send(self.__target_system,
                                          self.__target_component,
                                          mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                                          0,
                                          mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                                          mode_id,
                                          0, 0, 0, 0, 0)
        # Confirmation Message
        self.confirmation_message(f'Setting Mode to {mode}')
        
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
        :return: nothing
        """
        # Changing mode to 'GUIDED'
        self.change_mode(mode="GUIDED")

        # Arming
        self.arm(skip_prearm_checks=skip_prearm_checks, wait_armable=wait_armable)
        
        if takeoff_altitude is None or takeoff_altitude == 0:
            return
        # Takeoff
        print(f'Taking off to altitude of {takeoff_altitude} metres...')
        self.__conn.mav.command_long_send(
            self.__target_system,
            self.__target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0, 0, 0, 0, 0, 0, takeoff_altitude)
        # Confirmation Message
        msg = self.__conn.recv_match(type="COMMAND_ACK", blocking=True)
        print(msg)
        # Altitude Checking
        while True:
            msg = self.__conn.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
            if verbose:
                print(f"Relative Altitude: {msg.relative_alt / 1000}")
            if msg.relative_alt > takeoff_altitude * 1000 * 0.98:
                break
            time.sleep(1)
        # Target reached message
        print(f"Reached target altitude of {takeoff_altitude} metres!")
    
    def land(self, verbose=True):
        """
        Lands the vehicle on the ground. Might disarm it, depending on the command given.

        :param verbose: Controls display of altitude update messages
        :return: nothing
        """
        # Landing
        print(f'Landing...')
        self.__conn.mav.command_long_send(
            self.__target_system,
            self.__target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0,
            0, 0, 0, 0, 0, 0, 0)
        # Confirmation Message
        msg = self.__conn.recv_match(type="COMMAND_ACK", blocking=True)
        print(msg)
        """
        # Altitude Checking
        while True:
            msg = self.__conn.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
            if verbose:
                print(f"Relative Altitude: {msg.relative_alt / 1000}")
            if msg.relative_alt < 10:
                break
            time.sleep(1)
        """
        print(f'Vehicle Landed!')
        # Disarming
        # if disarm:
        #     self.disarm()
            
    def goto_waypoint(self, target_lat, target_lon, target_alt=None, verbose=True):
        """
        Goes to a waypoint, based on the global latitude and longitude coordinates. The target altitude is relative to the home location.

        :param target_lat: Type (float). It is the unscaled latitude value of the target waypoint.
        :param target_lon: Type (float). It is the unscaled longitude value of the target waypoint.
        :param target_alt: Type (int). It is the target altitude, relative to the home location, in metres.
        :param verbose:    Controls display of location update messages.
        :return: nothing
        """
        # Scaling of coordinates
        if target_alt is None:
            msg = self.__conn.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
            target_alt = msg.relative_alt // 1000
        target_lat = int(target_lat * 1e7)
        target_lon = int(target_lon * 1e7)
        # Command for location change
        msg = mavutil.mavlink.MAVLink_set_position_target_global_int_message(0, self.__target_system, self.__target_component,
                                                                             mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                                                                             int(0b110111111000),
                                                                             target_lat, target_lon, target_alt,
                                                                             0, 0, 0, 0, 0, 0, 0, 0)
        self.__send_mavlink(msg)

        # Location checking
        print('Heading to waypoint!')
        start = False
        while True:
            msg = self.__conn.recv_match(type='NAV_CONTROLLER_OUTPUT', blocking=True)
            if verbose:
                print(f'Distance to waypoint: {msg.wp_dist}')
            if msg.wp_dist > 0:
                start = True
            elif msg.wp_dist == 0 and start:
                break
            time.sleep(3)
        # Confirmation Message
        print("\nWaypoint reached!!!\n")

    def change_altitude_to(self, target_alt, verbose=False):
        """
        Changes altitude of the vehicle. Target altitude is given with respect to the home location.

        :param target_alt: Type (int). It is the target altitude, relative to the home location, in metres.
        :param verbose:    Controls display of altitude update messages.
        :return: nothing
        """
        # Command for altitude change
        print(f'Changing altitude to {target_alt} metres above home location')

        loc = self.get_location()
        target_lat = int(loc['lat'] * 1e7)
        target_lon = int(loc['lon'] * 1e7)

        # Command for location change
        msg = mavutil.mavlink.MAVLink_set_position_target_global_int_message(0, self.__target_system, self.__target_component,
                                                                             mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                                                                             int(0b110111111000),
                                                                             target_lat, target_lon, target_alt,
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
        print(f"Moving {north_displacement} metres North, {east_displacement} metres East, {up_displacement} metres UP...")
        self.__conn.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(0, self.__target_system, self.__target_component,
                                                                                           mavutil.mavlink.MAV_FRAME_BODY_NED,
                                                                                           int(0b110111111000),
                                                                                           north_displacement, east_displacement, up_displacement * -1,
                                                                                           0, 0, 0, 0, 0, 0, 0, 0))
        # Location checking
        start = False
        while True:
            msg = self.__conn.recv_match(type='NAV_CONTROLLER_OUTPUT', blocking=True)
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
        msg = mavutil.mavlink.MAVLink_set_position_target_local_ned_message(0, self.__target_system, self.__target_component,
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
        print(f'Moved up {difference_altitude} metres!') if difference_altitude > 0 else print(f'Moved down {difference_altitude} metres!')

    def move_ned(self, dist_north: float, dist_east: float, dist_down: float = 0, max_axis_speed: float = 1):
        """
        Moves the vehicle using velocity decomposition, based on North, East and Down directions. Units are in metres.

        :param dist_north: Type (int). Distance to move in North direction in metres.
        :param dist_east: Type (int). Distance to move in East direction in metres.
        :param dist_down: Type (int). Distance to move down in metres. [Down -> Positive, Up -> Negative]
        :param max_axis_speed: Maximum allowed speed in any NED axis, in m/s.
        :return: nothing
        """
        duration_north, duration_east, duration_down = abs(dist_north / max_axis_speed), abs(dist_east / max_axis_speed), abs(dist_down / max_axis_speed)
        duration = max(duration_north, duration_east, duration_down)
        duration = math.ceil(duration)
        velocity_north, velocity_east, velocity_down = dist_north / duration, dist_east / duration, dist_down / duration

        self.__send_global_velocity(velocity_north, velocity_east, velocity_down, duration=duration)
        self.__send_global_velocity(0, 0, 0, 1)

    def __send_global_velocity(self, velocity_north, velocity_east, velocity_down, duration):
        duration = int(duration)
        msg = mavutil.mavlink.MAVLink_set_position_target_local_ned_message(0,
                                                                            self.__target_system, self.__target_component,
                                                                            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                                                                            int(0b111111000111),
                                                                            0, 0, 0,   # Distances
                                                                            velocity_north,
                                                                            velocity_east,
                                                                            velocity_down,
                                                                            0, 0, 0,  # Accelerations
                                                                            0, 0   # Yaw, Yaw-rate
                                                                            )

        # Send command to vehicle on 1 Hz cycle
        for x in range(0, duration):
            # Command for velocity change
            self.__conn.mav.send(msg)
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

        duration_north, duration_east, duration_down = abs(dist_north / max_axis_speed), abs(dist_east / max_axis_speed), abs(dist_down / max_axis_speed)
        duration = max(duration_north, duration_east, duration_down)
        duration = max(1, math.ceil(duration))
        velocity_north, velocity_east, velocity_down = dist_north / duration, dist_east / duration, dist_down / duration

        self.__send_global_velocity(velocity_north, velocity_east, velocity_down, duration=duration)
        self.__send_global_velocity(0, 0, 0, 2)

    def mission(self, waypoints):
        wp = mavwp.MAVWPLoader()
        # print("Clearing all old commands")
        # msg = self.__conn.mav.mission_clear_all_encode(self.__target_system, self.__target_component)
        # self.__send_mavlink(msg)
        # print(mavutil.mavlink)
        # print(self.__conn.mav)

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

        print("Adding new commands")
        for mission_idx, waypoint in enumerate(waypoints):
            lat = waypoint['latitude']
            lon = waypoint['longitude']
            alt = waypoint['altitude']

            mission_cmd = self.__conn.mav.mission_item_encode(self.__target_system, self.__target_component,
                                                              mission_idx + 1,   # seq
                                                              mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,   # frame
                                                              mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,   # Command
                                                              0,   # Current
                                                              1,   # Auto-continue
                                                              0, 0, 0, math.nan,
                                                              lat, lon, alt
                                                              )
            wp.add(mission_cmd)
            # self.__send_mavlink(mission_cmd)

        wp.add(mission_cmd)

        # send waypoint to airframe
        self.__conn.waypoint_clear_all_send()
        self.__conn.waypoint_count_send(wp.count())

        waypoints_count = wp.count()
        for i in range(wp.count()):
            msg = self.__conn.recv_match(type=['MISSION_REQUEST'], blocking=True)
            print(msg)
            self.__conn.mav.send(wp.wp(msg.seq))
            print(f'Sending waypoint {msg.seq}')

        #msg = self.__conn.mav.mission_set_current_encode(self.__target_system, self.__target_component, 0)
        #self.__send_mavlink(msg)

        msg = self.__conn.recv_match(type='MISSION_ACK', blocking=True)
        print("Mission acknowledged!")
        print(msg)

        print("Changing mode to AUTO...")
        self.change_mode('AUTO')

        messages = ['MISSION_CURRENT', 'MISSION_ITEM_REACHED']

        self.configure_message_stream(message_ids=[mavutil.mavlink.MAVLINK_MSG_ID_MISSION_ITEM_REACHED,
                                                   mavutil.mavlink.MAVLINK_MSG_ID_MISSION_CURRENT],
                                      time_duration=1)

        for i in range(60):
            self.get_heading()
            msg = self.__conn.recv_match(type=['MISSION_ITEM_REACHED'], blocking=True)
            if msg is not None:
                print(i, msg)

            msg = self.__conn.recv_match(type=['MISSION_CURRENT'], blocking=True)
            if msg is not None:
                print(i, msg)

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
                                 1,   # Speed type
                                 ground_speed,   # Ground speed
                                 -1,   # Indicates no change in throttle
                                 0, 0, 0, 0)
        self.__send_mavlink(msg)
        # Confirmation Message
        msg = self.__conn.recv_match(type="COMMAND_ACK", blocking=True)
        print(msg)
        return

    def __set_air_speed(self, air_speed):
        msg = self.__long_encode(mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
                                 0,
                                 0,   # Speed type
                                 air_speed,   # Air speed
                                 -1,   # Indicates no change in throttle
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

    def get_location(self, relative_frame: bool = True):
        msg = self.__conn.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        lat, lon, alt = msg.lat / 1e7, msg.lon / 1e7, msg.alt / 1000
        if relative_frame:
            alt = alt - self.home_location['alt']
        current_location = {"lat": lat,
                            "lon": lon,
                            "alt": alt}
        return current_location

    def get_attitude(self):
        print(f'Attitude of vehicle:')
        print(self.__conn.recv_match(type="AHRS", blocking=True))
        print(self.__conn.recv_match(type="AHRS2", blocking=True))
        print(self.__conn.recv_match(type="VFR_HUD", blocking=True))
        # print(self.__conn.recv_match(type="AHRS3", blocking=True))
        print()

    def get_heading(self):
        msg = self.__conn.recv_match(type="AHRS2", blocking=True)

        yaw = msg.yaw
        pitch = msg.pitch
        roll = msg.roll
        # print(msg)
        yaw_degrees = yaw * 180 / math.pi
        yaw_degrees = yaw_degrees % 360
        print(f"Yaw in degrees: {yaw_degrees}...")

        msg = self.__conn.recv_match(type="GLOBAL_POSITION_INT", blocking=True)
        yaw_degrees = msg.hdg / 100
        print(f"Yaw from global position: {yaw_degrees}...")
        return yaw_degrees

    def get_speed(self):
        print(f'Speed of vehicle:')
        msg = self.__conn.recv_match(type="VFR_HUD", blocking=True)
        print(msg, '\n')

    def get_ground_speed(self):
        msg = self.__conn.recv_match(type="VFR_HUD", blocking=True)
        ground_speed = msg.groundspeed
        return ground_speed

    def get_air_speed(self):
        msg = self.__conn.recv_match(type="VFR_HUD", blocking=True)
        air_speed = msg.airspeed
        return air_speed

    def set_servo(self, servo_number, servo_pwm):
        """
        Changes the pwm of the servo. Used for dropping of payload from the vehicle.

        :param servo_number: Servo ID to identify which servo's pwm to change.
                             If servo_n is the AUX port to set, servo_number is (servo_n + 8), offset by 8 MAIN outputs.
        :param servo_pwm:    Target pwm for the servo. This pulse-width is in microseconds.  [Between 1000 and 2000]
        :return: nothing
        """
        # Command for setting the servo pwm
        self.__conn.mav.command_long_send(self.__target_system, self.__target_component,
                                          mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                                          0,
                                          servo_number, servo_pwm,
                                          0, 0, 0, 0, 0)
        # Confirmation Message
        self.confirmation_message(task='Servo Command')



    def goto_frd_position(self, forward_displacement, right_displacement, up_displacement, verbose=True):
        """
        Changes the position of the vehicle, based on Forward, Right and Down directions. Units are in metres.

        :param forward_displacement: Type (int). Distance to move in Forward direction in metres.
        :param right_displacement:  Type (int). Distance to move in Right direction in metres.
        :param up_displacement:    Type (int). Distance to move up in metres. [Up -> Positive, Down -> Negative]
        :param verbose:            Controls display of location update messages.
        :return: nothing
        """
        down_displacement = up_displacement * -1
        # Command for location change
        print(f"Moving {forward_displacement} metres Forward, {right_displacement} metres Right, {up_displacement} metres UP...")
        msg = mavutil.mavlink.MAVLink_set_position_target_local_ned_message(0, self.__target_system, self.__target_component,
                                                                            mavutil.mavlink.MAV_FRAME_BODY_FRD,
                                                                            int(0b110111111000),
                                                                            forward_displacement, right_displacement, down_displacement,
                                                                            0, 0, 0, 0, 0, 0, 0, 0)
        self.__send_mavlink(msg)
        # Location checking
        start = False
        while True:
            msg = self.__conn.recv_match(type='NAV_CONTROLLER_OUTPUT', blocking=True)
            if verbose:
                print(f'Distance to target altitude: {msg.wp_dist} metres')
            if msg.wp_dist > 0:
                start = True
            elif msg.wp_dist == 0 and start:
                break
            time.sleep(3)
        # Confirmation Message
        print(f'New position reached!')
        print(f"Moved {forward_displacement} metres forward, {right_displacement} metres East, {up_displacement} metres UP!")
