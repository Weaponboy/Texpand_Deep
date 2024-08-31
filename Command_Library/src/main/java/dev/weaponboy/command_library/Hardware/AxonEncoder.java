package dev.weaponboy.command_library.Hardware;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogInputController;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class AxonEncoder {

    AnalogInput encoder;

    public void init(HardwareMap hardwareMap, String deviceName){
        encoder = hardwareMap.get(AnalogInput.class, deviceName);
    }

    public double getPosition(){
        return encoder.getVoltage() / 3.3 * 360;
    }

}
