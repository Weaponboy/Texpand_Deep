package org.firstinspires.ftc.teamcode.Testing.Vision;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.opencv.core.Point;

import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;
import dev.weaponboy.command_library.Subsystems.Collection;
import dev.weaponboy.nexus_pathing.PathingUtility.RobotPower;

@TeleOp
public class globalVisionTesting extends OpModeEX {

    @Override
    public void initEX() {
        delivery.Scanning.execute();

        odometry.startPosition(79.5,100,0);
    }

    @Override
    public void loopEX() {

        if (currentGamepad1.b && !lastGamepad1.b && collection.getCurrentCommand() != collection.autoCollectGlobal){
            collection.queueCommand(collection.autoCollectGlobal(new RobotPower(odometry.X(), odometry.Y(), odometry.Heading())));
        }else if (currentGamepad1.b && !lastGamepad1.b){
            collection.overrideCurrent(true, collection.defaultCommand);
        }

        if (currentGamepad1.a && !lastGamepad1.a){
            collection.queueCommand(collection.collect);
        }

        if (currentGamepad1.x && !lastGamepad1.x && collection.sampleSorterContour.isScanning()){
            collection.sampleSorterContour.setScanning(false);
            collection.targetPointGlobal = collection.sampleSorterContour.convertToFieldCoor(new RobotPower(odometry.X(), odometry.Y(), odometry.Heading()));
        }else if (currentGamepad1.x && !lastGamepad1.x && !collection.sampleSorterContour.isScanning()){
            collection.sampleSorterContour.setScanning(true);
        }

        if (!collection.sampleSorterContour.isScanning()){
            collection.portal.stopStreaming();
        }else {
            collection.portal.resumeStreaming();
        }

//        telemetry.addData("slides position ", collection.horizontalMotor.getCurrentPosition()/(440/35));
//        telemetry.addData("rail position", collection.getRailPosition());
        telemetry.addData("Loop time", loopTime);
        telemetry.addData("X", odometry.X());
        telemetry.addData("Y", odometry.Y());
        telemetry.addData("Heading", odometry.Heading());
        telemetry.addData("slides target ", collection.slideTargetPosition);
        telemetry.addData("rail target", collection.railTarget);
        telemetry.addData("Target point", collection.targetPointGlobal);
        telemetry.addData("collection.sampleSorterContour.isScanning()", collection.sampleSorterContour.isScanning());
        telemetry.update();
    }

    @Override
    public void stop() {

    }

}
