package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;
import dev.weaponboy.command_library.Hardware.ServoDegrees;

@TeleOp
public class servopoztest extends OpMode {
    public ServoDegrees fourBarMainPivot = new ServoDegrees();
    public ServoDegrees fourBarSecondPivot= new ServoDegrees();


    public ServoDegrees mainPivot=new ServoDegrees();
    public ServoDegrees secondPivot = new ServoDegrees();


    double CollectSecondPivot = 320;
    double CollectMainPivot = 75;
    double transferMainPivot = 210;
    double transferSecondPivot = 122;
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






    public TouchSensor ClawSensor;
    public TouchSensor clawIR;





    @Override
    public void init() {

        fourBarMainPivot.initServo("fourBarMainPivot",hardwareMap);
        fourBarSecondPivot.initServo("fourBarSecondPivot",hardwareMap);

        mainPivot.initServo("mainPivot",hardwareMap);
        secondPivot.initServo("secondPivot",hardwareMap);

        ClawSensor = hardwareMap.get(TouchSensor.class, "CollectionReset   ");
        clawIR = hardwareMap.get(TouchSensor.class, "DeliveryReset   ");



        mainPivot.setRange(335);
        secondPivot.setRange(335);


        fourBarMainPivot.setRange(335);
        fourBarSecondPivot.setRange(335);
        fourBarSecondPivot.setOffset(-20);
        fourBarMainPivot.setOffset(10);
        fourBarSecondPivot.setPosition(transferSecondPivot);
        fourBarMainPivot.setPosition(transferMainPivot);


        secondPivot.setPosition(secondPivotTransfer);
        mainPivot.setPosition(mainPivotTransfer);


    }

    @Override
    public void loop() {
        telemetry.addData("intakeClaw", ClawSensor.isPressed());
        telemetry.addData("depoIR",clawIR.isPressed());
        telemetry.update();

    }
}
