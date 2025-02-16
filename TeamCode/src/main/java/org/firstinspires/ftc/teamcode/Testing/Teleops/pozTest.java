package org.firstinspires.ftc.teamcode.Testing.Teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import dev.weaponboy.command_library.Hardware.ServoDegrees;

//@TeleOp
public class pozTest extends OpMode {

    public ServoDegrees fourBarMainPivot = new ServoDegrees();
    public ServoDegrees fourBarSecondPivot= new ServoDegrees();
    public ServoDegrees turret =new ServoDegrees();

    double mainPivotHang = 83;
    double secondPivotHang = 210;

    @Override
    public void init() {
        fourBarMainPivot.initServo("fourBarMainPivot", hardwareMap);
        fourBarSecondPivot.initServo("fourBarSecondPivot", hardwareMap);
        turret.initServo("linearRailServo", hardwareMap);

        fourBarMainPivot.setRange(335);
        fourBarSecondPivot.setRange(335);

        fourBarMainPivot.setOffset(4);
        fourBarSecondPivot.setOffset(-5);

        turret.setRange(335);

        turret.setOffset(-4);

        turret.setDirection(Servo.Direction.REVERSE);

        //inset it in
//        turret.setPosition(42);

        //hold in
        turret.setPosition(57);

    }

    @Override
    public void loop() {
        fourBarMainPivot.setPosition(mainPivotHang);
        fourBarSecondPivot.setPosition(secondPivotHang);
    }

}
