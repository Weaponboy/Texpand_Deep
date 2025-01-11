package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;
import dev.weaponboy.command_library.Hardware.MotorEx;
import dev.weaponboy.command_library.Hardware.ServoDegrees;

@TeleOp
public class servopoztest extends OpMode {
    public ServoDegrees fourBarMainPivot = new ServoDegrees();
    public ServoDegrees fourBarSecondPivot= new ServoDegrees();
    public ServoDegrees griperRotate= new ServoDegrees();

    public ServoDegrees PTO = new ServoDegrees();

    MotorEx hangPower = new MotorEx();

    public ServoDegrees mainPivot=new ServoDegrees();
    public ServoDegrees secondPivot = new ServoDegrees();
    public ServoDegrees deliveryGrip = new ServoDegrees();

    public ServoDegrees gripServo = new ServoDegrees();

    public ServoDegrees linerRailServo=new ServoDegrees();

    double CollectSecondPivot = 320;
    double CollectMainPivot = 75;
    double transferMainPivot = 210;
    double transferSecondPivot = 122;
    double transferIntMainPivot = 100;
    double transferIntSecondPivot = 190;
    double stowMainPivot = 180;
    double stowSecondPivot = 180;
    double transferUpMainPivot = 250;
    double transferUpSecondPivot = 145;
    double preCollectMainPivot = 105;
    double preCollectSecondPivot = 305;
    double preCollectChamberMainPivot =120;
    double preCollectChamberSecondPivot =310;

//4.9cm
    double secondPivotBehindTransfer = 70;
    double mainPivotBehindTransfer =278;

    double secondPivotTransfer = 115;
    double mainPivotTransfer = 205;

    double secondPivotBucket =240;
    double mainPivotBucket =100;

    double secondPivotScan = 160;
    double mainPivotScan = 160;

    double preMainClip = 100;
    double preSecondClip = 210;

    double mainClip = 140;
    double secondClip = 220;

    public TouchSensor ClawSensor;
    public TouchSensor clawIR;

    @Override
    public void init() {

//        fourBarMainPivot.initServo("fourBarMainPivot",hardwareMap);
//        fourBarSecondPivot.initServo("fourBarSecondPivot",hardwareMap);
//        gripServo.initServo("gripServo", hardwareMap);
//        griperRotate.initServo("gripperRotate", hardwareMap);
        deliveryGrip.initServo("devClaw", hardwareMap);
//
        deliveryGrip.setRange(180);
//
//        PTO.initServo("hangPTO", hardwareMap);
//        PTO.setRange(new PwmControl.PwmRange(600, 2500), 270);
//        gripServo.setRange(180);
//        griperRotate.setRange(new PwmControl.PwmRange(500, 2500), 180);
//
//        hangPower.initMotor("hangPower", hardwareMap);
//        hangPower.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        PTO.setDirection(Servo.Direction.REVERSE);
//
//        mainPivot.initServo("mainPivot",hardwareMap);
//        secondPivot.initServo("secondPivot",hardwareMap);
//
        ClawSensor = hardwareMap.get(TouchSensor.class, "clawsensor");
        clawIR = hardwareMap.get(TouchSensor.class, "DeliveryReset");
//        linerRailServo.initServo("linearRailServo", hardwareMap);
//
//        mainPivot.setRange(335);
//        secondPivot.setRange(335);
//        linerRailServo.setRange(1800);
//
//        griperRotate.setDirection(Servo.Direction.REVERSE);
//        griperRotate.setOffset(10);
////        griperRotate.setPosition(0);
//
        deliveryGrip.setOffset(0);
//
//        fourBarMainPivot.setRange(335);
//        fourBarSecondPivot.setRange(335);
//        fourBarSecondPivot.setOffset(-20);
//        fourBarMainPivot.setOffset(10);
//
//        fourBarSecondPivot.setPosition(175);
//        fourBarMainPivot.setPosition(158);
//
////        secondPivot.setPosition(preSecondClip);
////        mainPivot.setPosition(120);
//
//        mainPivot.setPosition(mainPivotTransfer);
//        secondPivot.setPosition(secondPivotTransfer);

        //straight down = 271
        //parallel to hte ground = 190.5
        //188
        //210
        //120

        deliveryGrip.setPosition(120);


    }

    @Override
    public void loop() {

//        if (gamepad1.start){
//            gripServo.setPosition(45);
//        }
//
//        if (gamepad1.a){
//            gripServo.setPosition(55);
//        }
//
//        if (gamepad1.back){
//            gripServo.setPosition(100);
//        }
//
//        if (gamepad1.left_bumper){
////
//            mainPivot.setPosition(mainClip);
//            secondPivot.setPosition(secondClip);
////            deliveryGrip.disableServo();
//        }
//
//        if (gamepad1.dpad_left){
//            fourBarSecondPivot.setPosition(150);
//            fourBarMainPivot.setPosition(190);
//        }
//
//        if (gamepad1.dpad_right){
//            fourBarSecondPivot.setPosition(138);
//            fourBarMainPivot.setPosition(200);
//        }
//
        if (gamepad1.dpad_up){
            deliveryGrip.setPosition(102);
        }

        if (gamepad1.dpad_down){
            deliveryGrip.setPosition(145);
        }

//        double power = 0;
//
//        if (gamepad1.right_bumper){
//            power = -1;
//        } else if (gamepad1.left_bumper) {
//            power = 1;
//        }else {
//            power = 0;
//        }
//
//        if (gamepad1.dpad_up){
//            setRailTargetPosition(13);
//        } else if (gamepad1.dpad_left) {
//            setRailTargetPosition(0);
//        }else if (gamepad1.dpad_right){
//            setRailTargetPosition(26);
//        }
//
//        hangPower.update(power);

//        fourBarMainPivot.setPosition(transferIntMainPivot);
//
//        try {
//            Thread.sleep(100);
//        } catch (InterruptedException e) {
//            throw new RuntimeException(e);
//        }
//
//        fourBarSecondPivot.setPosition(transferIntSecondPivot);
//


        telemetry.addData("intakeClaw", ClawSensor.isPressed());
        telemetry.addData("depoIR",clawIR.isPressed());
        telemetry.update();

    }

    public void setRailTargetPosition(double targetPosition) {
        double degreesPerCM = (double) 900 / 26;

        double servoTarget = 450+(degreesPerCM*targetPosition);

        if (servoTarget < 450){
            linerRailServo.setPosition(450);
        } else if (servoTarget > 1350) {
            linerRailServo.setPosition(1350);
        }else {
            linerRailServo.setPosition(servoTarget);
        }

    }

}
