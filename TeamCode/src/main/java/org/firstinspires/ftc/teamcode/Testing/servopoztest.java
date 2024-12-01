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

    public ServoDegrees gripServo = new ServoDegrees();

    double CollectSecondPivot = 320;
    double CollectMainPivot = 75;
    double transferMainPivot = 210;
    double transferSecondPivot = 122;
    double transferIntMainPivot = 85;
    double transferIntSecondPivot = 210;
    double stowMainPivot = 180;
    double stowSecondPivot = 180;
    double transferUpMainPivot = 250;
    double transferUpSecondPivot = 145;
    double preCollectMainPivot =95;
    double preCollectSecondPivot =310;
    double preCollectChamberMainPivot =120;
    double preCollectChamberSecondPivot =310;

//4.9cm
    double secondPivotBehindTransfer = 70;
    double mainPivotBehindTransfer =278;
    double secondPivotTransfer = 70;
    double mainPivotTransfer =260;
    double secondPivotBucket =240;
    double mainPivotBucket =100;
    double secondPivotScan =170;
    double mainPivotScan =160;
    double preMainClip = 182;
    double preSecondClip =150;
    double mainClip = 185;
    double secondClip =100;

    public TouchSensor ClawSensor;
    public TouchSensor clawIR;

    @Override
    public void init() {

        fourBarMainPivot.initServo("fourBarMainPivot",hardwareMap);
        fourBarSecondPivot.initServo("fourBarSecondPivot",hardwareMap);
        gripServo.initServo("gripServo", hardwareMap);
        griperRotate.initServo("gripperRotate", hardwareMap);

        PTO.initServo("hangPTO", hardwareMap);
        PTO.setRange(new PwmControl.PwmRange(600, 2500), 270);
        gripServo.setRange(180);
        griperRotate.setRange(new PwmControl.PwmRange(500, 2500), 180);

        hangPower.initMotor("hangPower", hardwareMap);
        hangPower.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        PTO.setDirection(Servo.Direction.REVERSE);

        mainPivot.initServo("mainPivot",hardwareMap);
        secondPivot.initServo("secondPivot",hardwareMap);

        ClawSensor = hardwareMap.get(TouchSensor.class, "CollectionReset");
        clawIR = hardwareMap.get(TouchSensor.class, "DeliveryReset");

        mainPivot.setRange(335);
        secondPivot.setRange(335);

        griperRotate.setDirection(Servo.Direction.REVERSE);
        griperRotate.setOffset(10);
        griperRotate.setPosition(0);

//        fourBarMainPivot.setRange(335);
//        fourBarSecondPivot.setRange(335);
//        fourBarSecondPivot.setOffset(-20);
//        fourBarMainPivot.setOffset(10);
//        fourBarSecondPivot.setPosition(transferIntSecondPivot);
//        fourBarMainPivot.setPosition(transferIntMainPivot);


//        secondPivot.setPosition(preSecondClip);
////        mainPivot.setPosition(120);
        mainPivot.setPosition(271);
        //straight down = 271
        //parallel to hte ground = 190.5
        //188
        //210
        //120


    }

    @Override
    public void loop() {

        if (gamepad1.start){
//            PTO.setPosition(145);
            gripServo.setPosition(53);
        }

        if (gamepad1.back){
            gripServo.setPosition(120);
        }

        double power = 0;

        if (gamepad1.right_bumper){
            power = -1;
        } else if (gamepad1.left_bumper) {
            power = 1;
        }else {
            power = 0;
        }

        hangPower.update(power);

        telemetry.addData("intakeClaw", ClawSensor.isPressed());
        telemetry.addData("depoIR",clawIR.isPressed());
        telemetry.update();

    }
}
