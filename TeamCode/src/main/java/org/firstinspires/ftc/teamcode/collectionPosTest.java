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
    public ServoDegrees hang1 = new ServoDegrees();

    public ServoDegrees mainPivot = new ServoDegrees();
    public ServoDegrees secondPivot = new ServoDegrees();

    public TouchSensor ClawSensor;
    public TouchSensor clawIR;

    @Override
    public void init() {

        fourBarMainPivot.initServo("fourBarMainPivot",hardwareMap);
        fourBarSecondPivot.initServo("fourBarSecondPivot",hardwareMap);
        hang1.initServo("hang1",hardwareMap);
        gripServo.initServo("gripServo", hardwareMap);
        griperRotate.initServo("gripperRotate", hardwareMap);

        mainPivot.initServo("mainPivot",hardwareMap);
        secondPivot.initServo("secondPivot",hardwareMap);

        mainPivot.setRange(335);
        secondPivot.setRange(335);

        turret.initServo("linearRailServo", hardwareMap);

        turret.setRange(335);
        fourBarMainPivot.setRange(335);
        fourBarSecondPivot.setRange(335);

        mainPivot.setOffset(4.9);
        mainPivot.setPosition(250);
        secondPivot.setPosition(107);

        ClawSensor = hardwareMap.get(TouchSensor.class, "CollectionReset");
        clawIR = hardwareMap.get(TouchSensor.class, "DeliveryReset");

        turret.setOffset(3.5);
        fourBarMainPivot.setOffset(4);
        fourBarSecondPivot.setOffset(-5);

        turret.setPosition(167.5);

        fourBarSecondPivot.setPosition(128);
        fourBarMainPivot.setPosition(182);

        griperRotate.setRange(new PwmControl.PwmRange(500, 2500), 180);
        gripServo.setRange(180);

        griperRotate.setDirection(Servo.Direction.REVERSE);
        griperRotate.setOffset(10);
        griperRotate.setPosition(180);
    }

    @Override
    public void loop() {

        telemetry.addData("intakeClaw", ClawSensor.isPressed());
        telemetry.addData("depoIR",clawIR.isPressed());
        telemetry.update();

    }

}