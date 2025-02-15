package com.example.sriram;

import java.io.IOException;
import java.net.Socket;
import java.util.ArrayList;

import io.dronefleet.mavlink.MavlinkConnection;
import io.dronefleet.mavlink.common.CommandAck;
import io.dronefleet.mavlink.common.CommandLong;
import io.dronefleet.mavlink.common.GlobalPositionInt;
import io.dronefleet.mavlink.common.MavCmd;
import io.dronefleet.mavlink.common.MavFrame;
import io.dronefleet.mavlink.common.MavMissionType;
import io.dronefleet.mavlink.common.MissionAck;
import io.dronefleet.mavlink.common.MissionClearAll;
import io.dronefleet.mavlink.common.MissionCount;
import io.dronefleet.mavlink.common.MissionCurrent;
import io.dronefleet.mavlink.common.MissionItemInt;
import io.dronefleet.mavlink.common.MissionRequest;
import io.dronefleet.mavlink.common.NavControllerOutput;
import io.dronefleet.mavlink.common.PositionTargetTypemask;
import io.dronefleet.mavlink.common.SetPositionTargetGlobalInt;
import io.dronefleet.mavlink.common.SetPositionTargetLocalNed;

public class Drone {
    private final String ip;
    private final int port;
    private final Socket socket;
    private final MavlinkConnection connection;
    private final int system_id = 1;
    private final int component_id = 0;

    public Drone(String IP, int port) throws IOException {
        this.ip = IP;
        this.port = port;
        this.socket = new Socket(this.ip, this.port);
        this.connection = MavlinkConnection.create(socket.getInputStream(), socket.getOutputStream());
    }

    public Drone(String IP, String port) throws IOException {
        this.ip = IP;
        this.port = Integer.parseInt(port);
        this.socket = new Socket(this.ip, this.port);
        this.connection = MavlinkConnection.create(socket.getInputStream(), socket.getOutputStream());
    }

    public String set_mode(Modes mode) throws IOException{
        CommandLong command = CommandLong.builder()
            .targetSystem(system_id)
            .targetComponent(component_id)
            .command(MavCmd.MAV_CMD_DO_SET_MODE)
            .param1(1)
            .param2(mode.getModeId())
            .build();

        connection.send1(system_id, component_id, command);

        CommandAck ack = getCommandAck();
        System.out.println("Received acknowledgment: " + ack);
        return ack.toString();  
    }

    public String arm() throws IOException {
        CommandLong command = CommandLong.builder()
            .targetSystem(system_id)
            .targetComponent(component_id)
            .command(MavCmd.MAV_CMD_COMPONENT_ARM_DISARM)
            .param1(1)
            .param2(0)
            .build();

        connection.send1(system_id, component_id, command);

        CommandAck ack = getCommandAck();
        System.out.println("Received acknowledgment for Arming: " + ack);
        return ack.toString(); 
    }

    public String disarm(boolean force_disarm) throws IOException {
        int param2 = 0;
        if (force_disarm) {
            param2 = 21196;
        }
        CommandLong command = CommandLong.builder()
            .targetSystem(system_id)
            .targetComponent(component_id)
            .command(MavCmd.MAV_CMD_COMPONENT_ARM_DISARM)
            .param1(0)
            .param2(param2)
            .build();

        connection.send1(system_id, component_id, command);

        CommandAck ack = getCommandAck();
        System.out.println("Received acknowledgment: " + ack);
        return ack.toString(); 
    }

    public String takeoff(float target_altitude) throws IOException, InterruptedException {
        CommandLong command = CommandLong.builder()
            .targetSystem(system_id)
            .targetComponent(component_id)
            .command(MavCmd.MAV_CMD_NAV_TAKEOFF)
            .param7(target_altitude)
            .build();

        connection.send1(system_id, component_id, command);

        CommandAck ack = (CommandAck) getCommandAck();
        System.out.println("Received acknowledgment for Takeoff: " + ack);

        while (true) {
            GlobalPositionInt globalPosition = getPositionPacket();
            // System.out.println("Received position: " + globalPosition);
            if (globalPosition.relativeAlt() / 1000 > 0.95 * target_altitude) {
                break;
            }
        }
        return "Done";
    }

    public String land() throws IOException, InterruptedException {
        this.set_mode(Modes.LAND);

        while (true) {
            GlobalPositionInt globalPosition = getPositionPacket();
            System.out.println("Received position: " + globalPosition);
            if (globalPosition.relativeAlt() / 1000 < 0.1) {
                break;
            }
        }
        return "Done";
    }

    public Object getPacket(Class payloadType) throws IOException {
        while (true) {
            Object message = connection.next().getPayload();
            if (message.getClass().getName().equals(payloadType.getName())) {
                return message;
            }
        }
    }
    
    public CommandAck getCommandAck() throws IOException {
        return (CommandAck) getPacket(CommandAck.class);
    }

    public MissionAck getMissionAck() throws IOException {
        return (MissionAck) getPacket(MissionAck.class);
    }
  
    public MissionRequest getMissionItemRequest() throws IOException {
        return (MissionRequest) getPacket(MissionRequest.class);
    }

    public GlobalPositionInt getPositionPacket() throws IOException {
        return (GlobalPositionInt) getPacket(GlobalPositionInt.class);
    }

    public NavControllerOutput getNavControllerPacket() throws IOException {
        return (NavControllerOutput) getPacket(NavControllerOutput.class);
    }

    public void goto_waypoint(Float target_lat, Float target_lon, Float target_alt) throws IOException, InterruptedException {
        int  target_lat_int, target_lon_int;
        float target_alt_float;
        if (target_alt == null) {
            GlobalPositionInt globalPosition = getPositionPacket();
            target_alt_float = globalPosition.relativeAlt() / 1000;
        }
        else {
            target_alt_float = (float) (target_alt);
        }
        target_lat_int = (int) (float) (target_lat * 1e7);
        target_lon_int = (int) (float) (target_lon * 1e7);

        SetPositionTargetGlobalInt command = SetPositionTargetGlobalInt.builder()
            .timeBootMs(0)
            .targetSystem(system_id)
            .targetComponent(component_id)
            .coordinateFrame(MavFrame.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT)
            .typeMask(PositionTargetTypemask.POSITION_TARGET_TYPEMASK_AX_IGNORE,
                      PositionTargetTypemask.POSITION_TARGET_TYPEMASK_AY_IGNORE,
                      PositionTargetTypemask.POSITION_TARGET_TYPEMASK_AZ_IGNORE,
                      PositionTargetTypemask.POSITION_TARGET_TYPEMASK_VX_IGNORE,
                      PositionTargetTypemask.POSITION_TARGET_TYPEMASK_VY_IGNORE,
                      PositionTargetTypemask.POSITION_TARGET_TYPEMASK_VZ_IGNORE,
                      PositionTargetTypemask.POSITION_TARGET_TYPEMASK_YAW_IGNORE,
                      PositionTargetTypemask.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE,
                      PositionTargetTypemask.POSITION_TARGET_TYPEMASK_FORCE_SET)
            .latInt(target_lat_int)
            .lonInt(target_lon_int)
            .alt(target_alt_float)
            .build();
        /*
        CommandInt command = CommandInt.builder()
            .targetSystem(system_id)
            .targetComponent(component_id)
            .frame(MavFrame.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT)
            .command(MavCmd.MAV_CMD_NAV_WAYPOINT)
            .current(0)
            .autocontinue(0)
            .x(target_lat_int)
            .y(target_lon_int)
            .z(target_alt_int)
            .build();*/

        connection.send1(system_id, component_id, command);

        while (true) {
            GlobalPositionInt globalPosition = getPositionPacket();
            NavControllerOutput navController = getNavControllerPacket();
            System.out.println("Distance to waypoint: " + navController.wpDist());
            System.out.println("Position: " + globalPosition);
            System.out.println();
            if (navController.wpDist() == 0) {
                return;
            }
        }
    }


    public void move_ned(float x, float y, float z) throws IOException, InterruptedException {
        SetPositionTargetLocalNed command = SetPositionTargetLocalNed.builder()
            .timeBootMs(0)
            .targetSystem(system_id)
            .targetComponent(component_id)
            .coordinateFrame(MavFrame.MAV_FRAME_BODY_NED)
            .typeMask(PositionTargetTypemask.POSITION_TARGET_TYPEMASK_AX_IGNORE,
                      PositionTargetTypemask.POSITION_TARGET_TYPEMASK_AY_IGNORE,
                      PositionTargetTypemask.POSITION_TARGET_TYPEMASK_AZ_IGNORE,
                      PositionTargetTypemask.POSITION_TARGET_TYPEMASK_VX_IGNORE,
                      PositionTargetTypemask.POSITION_TARGET_TYPEMASK_VY_IGNORE,
                      PositionTargetTypemask.POSITION_TARGET_TYPEMASK_VZ_IGNORE,
                      PositionTargetTypemask.POSITION_TARGET_TYPEMASK_YAW_IGNORE,
                      PositionTargetTypemask.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE,
                      PositionTargetTypemask.POSITION_TARGET_TYPEMASK_FORCE_SET)
            .x(x)
            .y(y)
            .z(z)
            .build();

        connection.send1(system_id, component_id, command);

        while (true) {
            GlobalPositionInt globalPosition = getPositionPacket();
            NavControllerOutput navController = getNavControllerPacket();
            System.out.println("Distance to waypoint: " + navController.wpDist());
            System.out.println("Position: " + globalPosition);
            System.out.println("Controller: " + navController);
            System.out.println();
            if (navController.wpDist() == 0) {
                return;
            }
        }
    }

    public void clearMission() throws IOException {
        MissionClearAll missionClear = MissionClearAll.builder()
            .targetSystem(system_id)
            .targetComponent(component_id)
            .missionType(MavMissionType.MAV_MISSION_TYPE_MISSION)
            .build();
        
        System.out.println("Sending mission clear command.");

        connection.send1(system_id, component_id, missionClear);

        MissionAck ack = getMissionAck();
        System.out.println("Got acknowledgemnt for clearing mission. " + ack);
    }

    public void sendMissionCount(int count) throws IOException {
        MissionCount missionCount = MissionCount.builder()
            .targetSystem(system_id)
            .targetComponent(component_id)
            .count(count)
            .missionType(MavMissionType.MAV_MISSION_TYPE_MISSION)
            .build();

        System.out.println("Sending mission count.");
        connection.send1(system_id, component_id, missionCount);
    }

    public void sendMissionItem(Waypoint waypoint, int sequence) throws IOException {
        MissionItemInt missionItem = MissionItemInt.builder()
            .targetSystem(system_id)
            .targetComponent(component_id)
            .seq(sequence)
            .frame(MavFrame.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT)
            .command(MavCmd.MAV_CMD_NAV_WAYPOINT)
            .current(0)
            .autocontinue(1)
            .x(waypoint.getScaledLatitude())
            .y(waypoint.getScaledLongitude())
            .z(waypoint.altitude)
            .missionType(MavMissionType.MAV_MISSION_TYPE_MISSION)
            .build();

        //System.out.println("Sending Mission Item.");
        connection.send1(system_id, component_id, missionItem);
    }

    public void do_mission(ArrayList<Waypoint> arr) throws IOException, InterruptedException {
        MissionAck ack;
        int count = arr.size() + 2;
        int sequence = 0;
        Waypoint waypoint;
        MissionRequest request;
        NavControllerOutput navPacket;

        clearMission();

        sendMissionCount(count);

        request = getMissionItemRequest();
        System.out.println("Received mission item request. " + request);

        System.out.println("Sending duplicate of first missionItem. (Ignored)");
        waypoint = arr.get(0);
        sendMissionItem(waypoint, sequence++);

        for (Waypoint wp: arr) {
            request = getMissionItemRequest();
            System.out.println("Received mission item request. " + request);

            sendMissionItem(wp, sequence++);
        }

        request = getMissionItemRequest();
        System.out.println("Received mission item request. " + request);
        System.out.println("Sending duplicate of last missionItem. (For waiting mechanism)");
        waypoint = arr.get(arr.size()-1);
        sendMissionItem(waypoint, sequence++);    

        ack = getMissionAck();
        System.out.println("Received mission item acknowledgement. " + ack);

        System.out.println("Setting mode to Auto.");

        set_mode(Modes.AUTO);

        while (true) {
            MissionCurrent missionCurrent = (MissionCurrent) getPacket(MissionCurrent.class);
            System.out.println("Current mission item: " + missionCurrent.seq());
            // MissionItemReached missionItemReached = (MissionItemReached) getPacket(MissionItemReached.class);
            // System.out.println("Mission status: " + missionItemReached);
            navPacket = getNavControllerPacket();
            System.out.println("Distance to next waypoint: " + navPacket.wpDist());
            if (missionCurrent.seq() == count-1) {
                System.out.println("Reached final waypoint.");
                return;
            }
        }
    }
}
