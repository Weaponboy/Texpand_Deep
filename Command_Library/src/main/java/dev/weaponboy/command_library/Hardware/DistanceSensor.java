package dev.weaponboy.command_library.Hardware;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DistanceSensor {

    AnalogInput sensor;

    public void setOffset(double offset) {
        this.offset = offset;
    }

    double offset = 0;

    public void init(HardwareMap hardwareMap, String deviceName){
        sensor = hardwareMap.get(AnalogInput.class, deviceName);
    }

    public double getPosition(){
        return (sensor.getVoltage() / (3.3 / 1024) * 6) + offset;
    }

}
