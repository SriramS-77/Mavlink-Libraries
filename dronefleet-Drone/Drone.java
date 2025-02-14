package com.example.sriram;

import java.io.IOException;

import io.dronefleet.mavlink.MavlinkConnection;
import io.dronefleet.mavlink.common.CommandAck;
import io.dronefleet.mavlink.common.CommandLong;
import io.dronefleet.mavlink.common.GlobalPositionInt;
import io.dronefleet.mavlink.common.MavCmd;
import io.dronefleet.mavlink.common.MavFrame;
import io.dronefleet.mavlink.common.MissionAck;
import io.dronefleet.mavlink.common.NavControllerOutput;
import io.dronefleet.mavlink.common.PositionTargetTypemask;
import io.dronefleet.mavlink.common.SetPositionTargetGlobalInt;
import io.dronefleet.mavlink.common.SetPositionTargetLocalNed;

public class Drone {
    private MavlinkConnection connection;
    private final int system_id = 1;
    private final int component_id = 0;

    public Drone(MavlinkConnection connection) {
        this.connection = connection;
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
        System.out.println("Received acknowledgment: " + ack);
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
        System.out.println("Received acknowledgment: " + ack);

        while (true) {
            GlobalPositionInt globalPosition = getPositionPacket();
            System.out.println("Received position: " + globalPosition);
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
}
