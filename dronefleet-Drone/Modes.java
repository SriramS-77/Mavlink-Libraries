package com.example.sriram;

public enum Modes {
    STABILIZE(0),
    AUTO(3),
    GUIDED(4),
    LOITER(5),
    RTL(6),
    LAND(9),
    BRAKE(17),
    GUIDED_NO_GPS(20),
    SMART_RTL(21),
    ZIGZAG(24),
    AUTO_RTL(27);

    private final int mode_id;

    Modes (int mode_id) {
        this.mode_id = mode_id;
    }

    public int getModeId() {
        return this.mode_id;
    }
}
