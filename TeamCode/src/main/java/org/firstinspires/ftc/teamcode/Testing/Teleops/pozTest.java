package org.firstinspires.ftc.teamcode.Testing.Teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import dev.weaponboy.command_library.Hardware.ServoDegrees;

public class pozTest extends OpMode {
    public ServoDegrees fourBarMainPivot = new ServoDegrees();
    public ServoDegrees fourBarSecondPivot= new ServoDegrees();
    double mainPivotHang = 250;
    double secondPivotHang = 80;
    @Override
    public void init() {
        fourBarMainPivot.initServo("fourBarMainPivot", hardwareMap);
        fourBarSecondPivot.initServo("fourBarSecondPivot", hardwareMap);

        fourBarMainPivot.setRange(335);
        fourBarSecondPivot.setRange(335);

        fourBarMainPivot.setOffset(4);
        fourBarSecondPivot.setOffset(-5);

    }

    @Override
    public void loop() {
        fourBarMainPivot.setPosition(mainPivotHang);
        fourBarSecondPivot.setPosition(secondPivotHang);
    }
}
