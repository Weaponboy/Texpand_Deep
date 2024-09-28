package org.firstinspires.ftc.teamcode.Examples;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
public class AxonForthWire extends OpMode {

    AnalogInput servoWire;
    Servo servo;
    Servo servo2;

    FtcDashboard dashboard = FtcDashboard.getInstance();

    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    int counter = 0;

    boolean test = false;

    @Override
    public void init() {
//        servoWire = hardwareMap.get(AnalogInput.class, "servoWire");
        servo = hardwareMap.get(Servo.class, "servo");
        servo2 = hardwareMap.get(Servo.class, "servo2");

        servo.setPosition(0.5);
        servo2.setPosition(0.5);
    }

    @Override
    public void loop() {

//        dashboardTelemetry.addData("voltage", servoWire.getVoltage() / 3.3 * 360);
//        dashboardTelemetry.update();

        if (counter > 200){
            counter = 0;
//            if (servo.getPosition() > 0.5){
//                servo.setPosition(0);
//            } else if (servo.getPosition() < 0.5) {
//                servo.setPosition(1);
//            }
//            if (test){
//                servo.setPosition(1);
//                test = false;
//            } else if (!test) {
//                servo.setPosition(0);
//                test = true;
//            }

//            servo.setPosition(1);

        }

        counter++;

    }

}
