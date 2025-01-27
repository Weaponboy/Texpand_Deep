package org.firstinspires.ftc.teamcode.Testing.Slides;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;
import dev.weaponboy.command_library.Hardware.MotorEx;

@TeleOp
public class ExtendoPID extends OpMode {

    FtcDashboard dashboard = FtcDashboard.getInstance();

    public Telemetry dashboardTelemetry = dashboard.getTelemetry();

    public MotorEx horizontalMotor = new MotorEx();


    @Override
    public void init() {
        horizontalMotor.initMotor("horizontalMotor", hardwareMap);
    }

    @Override
    public void loop() {

        telemetry.addData("Position", horizontalMotor.getCurrentPosition());
        telemetry.update();
    }


}
