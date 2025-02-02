package dev.weaponboy.command_library.Hardware;

import dev.weaponboy.nexus_pathing.RobotUtilities.Vector2D;

public class SensorReadings {

    double sen1;
    double sen2;
    double sen3;

    public SensorReadings(double sen1, double sen2, double sen3){
        this.sen1 = sen1;
        this.sen2 = sen2;
        this.sen3 = sen3;
    }

    public double getSen1() {
        return sen1;
    }

    public double getSen2() {
        return sen2;
    }

    public double getSen3() {
        return sen3;
    }

}
