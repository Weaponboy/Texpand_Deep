package dev.weaponboy.command_library.Hardware;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DistanceSensor {

    AnalogInput sensor;

    public void init(HardwareMap hardwareMap, String deviceName){
        sensor = hardwareMap.get(AnalogInput.class, deviceName);
    }

    public double getPosition(){
        return (sensor.getVoltage() / (3.3 / 1024) * 6) - 300;
    }

}
