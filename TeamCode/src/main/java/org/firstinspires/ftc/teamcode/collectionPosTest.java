package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import dev.weaponboy.command_library.Hardware.MotorEx;
import dev.weaponboy.command_library.Hardware.ServoDegrees;

@TeleOp
public class collectionPosTest extends OpMode {

    //servos
    public ServoDegrees fourBarMainPivot = new ServoDegrees();
    public ServoDegrees fourBarSecondPivot= new ServoDegrees();
    public ServoDegrees griperRotate= new ServoDegrees();
    public ServoDegrees gripServo = new ServoDegrees();
    public ServoDegrees turret = new ServoDegrees();

    public TouchSensor ClawSensor;
    public TouchSensor clawIR;

    @Override
    public void init() {

        fourBarMainPivot.initServo("fourBarMainPivot",hardwareMap);
        fourBarSecondPivot.initServo("fourBarSecondPivot",hardwareMap);

        gripServo.initServo("gripServo", hardwareMap);
        griperRotate.initServo("gripperRotate", hardwareMap);

        turret.initServo("gripperRotate", hardwareMap);

        gripServo.setRange(180);
        griperRotate.setRange(new PwmControl.PwmRange(500, 2500), 180);

        ClawSensor = hardwareMap.get(TouchSensor.class, "clawsensor");
        clawIR = hardwareMap.get(TouchSensor.class, "DeliveryReset");

    }

    @Override
    public void loop() {

        telemetry.addData("intakeClaw", ClawSensor.isPressed());
        telemetry.addData("depoIR",clawIR.isPressed());
        telemetry.update();

    }

}
