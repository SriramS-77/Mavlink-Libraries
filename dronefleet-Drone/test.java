package com.example.sriram;

import java.io.IOException;
import java.net.Socket;

import io.dronefleet.mavlink.MavlinkConnection;


public class test {

    public static void main(String[] args) throws IOException {
        int system_id = 1;
        int component_id = 0;

        // Open a serial connection to the drone
        System.out.println("Sriram - hello!!!");

        try {
            //MavlinkConnection connection = MavlinkConnection.create(new FileInputStream(new File("/dev/ttyACM0")),
            //                                                new FileOutputStream(new File("/dev/ttyACM0")));
        
            Socket socket = new Socket("127.0.0.1", 12345);

            MavlinkConnection connection = MavlinkConnection.create(socket.getInputStream(), socket.getOutputStream());

            Drone drone = new Drone(connection);

            drone.set_mode(Modes.GUIDED);

            drone.arm();

            drone.takeoff(10);

            // drone.goto_waypoint(-35.3645384f, 149.1648531f, null);
            drone.move_ned(30, 30, -5);

            drone.land();

            Thread.sleep(5000);

            drone.disarm(false);

            /*
            System.out.println("Starting loop...");
            while (true)
            {
                // System.out.println("Connection's dialect: " + connection.getDialect(system_id));

                MavlinkMessage<?> message = connection.next();

                // System.out.println(message.getPayload());
                
                // System.out.println("Received message from: " + message.getOriginSystemId());
                // Handle battery status messages
                if (message.getPayload() instanceof BatteryStatus) {
                    BatteryStatus batteryStatus = (BatteryStatus) message.getPayload();
                    System.out.println("Battery voltage: " + batteryStatus.currentBattery() + " %");
                    System.out.println("Battery current: " + batteryStatus.batteryRemaining() + " %");
                }

                // Handle GPS position messages
                if (message.getPayload() instanceof GlobalPositionInt) {
                    GlobalPositionInt globalPosition = (GlobalPositionInt) message.getPayload();
                    System.out.println("Latitude: " + globalPosition.lat() / 1E7);
                    System.out.println("Longitude: " + globalPosition.lon() / 1E7);
                    System.out.println("Altitude: " + globalPosition.alt() / 1000.0 + " m");
                }
            }  */
        }
        catch (IOException e) {
            System.out.println("Exception got: " + e);
        } 
        catch (InterruptedException e) {
            System.out.println("Exception got: " + e);
        }
    }
}
