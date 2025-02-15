package com.example.sriram;

public class Waypoint {
    Float latitude;
    Float longitude;
    Float altitude;

    public Waypoint(float latitude, float longitude, float altitude) {
        this.latitude = latitude;
        this.longitude = longitude;
        this.altitude = altitude;
    }
    
    public Waypoint(float latitude, float longitude) {
        this.latitude = latitude;
        this.longitude = longitude;
        this.altitude = null;
    }

    public int getScaledLatitude() {
        return (int) (float) (this.latitude * 1e7);
    }

    public int getScaledLongitude() {
        return (int) (float) (this.longitude * 1e7);
    }
}
