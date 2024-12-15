package org.firstinspires.ftc.teamcode.Testing.Slides;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;

@TeleOp
public class ExtendoPID extends OpModeEX {

    FtcDashboard dashboard = FtcDashboard.getInstance();

    public Telemetry dashboardTelemetry = dashboard.getTelemetry();

    @Override
    public void initEX() {

    }

    @Override
    public void loopEX() {

        if (gamepad1.x){
            collection.setSlideTarget(40);
        }

        dashboardTelemetry.addData("Position", collection.getSlidePositionCM());
        dashboardTelemetry.addData("Target", collection.getSlideTarget());
        dashboardTelemetry.update();
    }


}
