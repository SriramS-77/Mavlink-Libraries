package com.example.sriram;

import java.io.IOException;
import java.util.ArrayList;


public class mission_test {
    public static void main(String[] args) throws IOException {
        String IP = "127.0.0.1";
        int port = 12345;

        System.out.println("Mission - hello!!!");

        ArrayList<Waypoint> arr = new ArrayList();
        arr.add(new Waypoint(-35.3640384f, 149.1650531f, 10));
        arr.add(new Waypoint(-35.3632384f, 149.1652531f, 10));

        try {
            //MavlinkConnection connection = MavlinkConnection.create(new FileInputStream(new File("/dev/ttyACM0")),
            //                                                new FileOutputStream(new File("/dev/ttyACM0")));
        
            Drone drone = new Drone(IP, port);

            drone.set_mode(Modes.GUIDED);

            drone.arm();

            drone.takeoff(10);

            drone.do_mission(arr);

            drone.land();

            Thread.sleep(5000);

            drone.disarm(false);
        }
        catch (IOException e) {
            System.out.println("Exception got: " + e);
        } 
        catch (InterruptedException e) {
            System.out.println("Exception got: " + e);
        }
    }
}
