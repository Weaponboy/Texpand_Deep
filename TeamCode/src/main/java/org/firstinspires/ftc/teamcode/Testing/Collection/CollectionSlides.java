package org.firstinspires.ftc.teamcode.Testing.Collection;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;
import dev.weaponboy.command_library.Hardware.DistanceSensor;
import dev.weaponboy.command_library.Hardware.MotorEx;
import dev.weaponboy.command_library.Subsystems.Collection;

@TeleOp(name = "PID_tuning", group = "Testing")
public class CollectionSlides extends OpModeEX {

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    @Override
    public void initEX() {

    }

    @Override
    public void loopEX() {

        if (gamepad1.a){
            collection.setSlideTarget(45);
        }

        if (gamepad1.b){
            collection.setSlideTarget(0);
        }

        if (gamepad1.y){
            collection.setSlideTarget(30);
        }

        dashboardTelemetry.addData("position", collection.getSlidePositionCM());
        dashboardTelemetry.addData("target", collection.getSlideTarget());
        dashboardTelemetry.update();
    }

}
