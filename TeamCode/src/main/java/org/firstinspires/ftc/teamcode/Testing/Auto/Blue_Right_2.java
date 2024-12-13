package org.firstinspires.ftc.teamcode.Testing.Auto;


import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;
import dev.weaponboy.command_library.Subsystems.Collection;
import dev.weaponboy.command_library.Subsystems.Delivery;
import dev.weaponboy.nexus_pathing.Follower.follower;
import dev.weaponboy.nexus_pathing.PathGeneration.commands.sectionBuilder;
import dev.weaponboy.nexus_pathing.PathGeneration.pathsManager;
import dev.weaponboy.nexus_pathing.PathingUtility.RobotPower;
import dev.weaponboy.nexus_pathing.RobotUtilities.Vector2D;
import dev.weaponboy.vision.SamplePipelines.findAngleUsingContour;

@Disabled
@Autonomous(name = "Blue Righ_2t", group = "Autos")
public class Blue_Right_2 extends OpModeEX {
    double targetHeading;


    pathsManager paths = new pathsManager();
    follower follow = new follower();


    private final sectionBuilder[] rightBluePath = {
            () -> paths.addPoints(new Vector2D(339, 160), new Vector2D(251, 170))

    };
    private final sectionBuilder[] colecting = {
            () -> paths.addPoints(new Vector2D(251, 170), new Vector2D(311,126))
    };
    private final sectionBuilder[] cliping = {
            () -> paths.addPoints(new Vector2D(83, 333), new Vector2D(211,319 ),new Vector2D(174, 252))
    };
    private final sectionBuilder[] cliping1 = {
            () -> paths.addPoints(new Vector2D(83, 333), new Vector2D(211,319 ),new Vector2D(178, 252))
    };
    private final sectionBuilder[] cliping2 = {
            () -> paths.addPoints(new Vector2D(83, 333), new Vector2D(211,319 ),new Vector2D(182, 252))
    };
    private final sectionBuilder[] cliping3 = {
            () -> paths.addPoints(new Vector2D(83, 333), new Vector2D(211,319 ),new Vector2D(186, 252))
    };
    private final sectionBuilder[] cliping4 = {
            () -> paths.addPoints(new Vector2D(83, 333), new Vector2D(211,319 ),new Vector2D(190, 252))
    };
    private final sectionBuilder[] cliping5 = {
            () -> paths.addPoints(new Vector2D(83, 333), new Vector2D(211,319 ),new Vector2D(194, 252))
    };

    public enum autoState {
        preLoad,
        clipping,
        collecting,
        obs_collect,
        finished
    }

    public enum building {
        built,
        notBuilt
    }

    boolean following = false;
    boolean queuedClipCommands = false;
    boolean detectingInSub = true;
    boolean busyDetecting = false;
    ElapsedTime detectingTimer = new ElapsedTime();

    autoState state = autoState.preLoad;
    building built = building.notBuilt;

    @Override
    public void initEX() {

        delivery.setGripperState(Delivery.gripper.grab);

        odometry.startPosition(341, 164, 180);

        paths.addNewPath("rightBluePath");
        paths.buildPath(rightBluePath);

        paths.addNewPath("colecting");
        paths.buildPath(colecting);

        paths.addNewPath("cliping");
        paths.buildPath(cliping);

        paths.addNewPath("cliping1");
        paths.buildPath(cliping1);

        paths.addNewPath("cliping2");
        paths.buildPath(cliping2);

        paths.addNewPath("clipin3");
        paths.buildPath(cliping3);

        paths.addNewPath("cliping4");
        paths.buildPath(cliping4);

        paths.addNewPath("cliping5");
        paths.buildPath(cliping5);

        follow.setPath(paths.returnPath("rightBluePath"));

        delivery.slideSetPoint(delivery.highChamberFront);
        delivery.slides = Delivery.slideState.moving;

        FtcDashboard.getInstance().startCameraStream(collection.sampleSorterContour, 30);

        collection.sampleSorterContour.setScanning(true);
        collection.sampleSorterContour.setTargetColor(findAngleUsingContour.TargetColor.red);
        collection.sampleSorterContour.closestFirst = true;
    }


    @Override
    public void loopEX() {

        if (state == autoState.preLoad) {

            if (built == building.notBuilt && delivery.slideMotor.getCurrentPosition() > 100) {
                follow.setPath(paths.returnPath("rightBluePath"));
                targetHeading = 180;
                following = true;
                built = building.built;
                delivery.PreClipFront.execute();
            }

            if (follow.isFinished(1, 4) && detectingInSub && delivery.getCurrentCommand() != delivery.clipFront){
                detectingInSub = false;
                busyDetecting = true;
                detectingTimer.reset();
//                delivery.mainPivot.setPosition(delivery.findCameraScanPosition());
            }
//
//            if (busyDetecting){
//                delivery.mainPivot.setPosition(delivery.findCameraScanPosition());
//            }

            if (follow.isFinished(1, 4) && !queuedClipCommands && !detectingInSub && detectingTimer.milliseconds() > 500) {

                queuedClipCommands = true;
                delivery.queueCommand(delivery.clipFront);
                delivery.queueCommand(delivery.clipFront);
                delivery.queueCommand(delivery.clipFront);
                delivery.queueCommand(delivery.clipFront);

            } else if (queuedClipCommands && collection.horizontalMotor.getCurrentPosition() < 40 && delivery.getSlidePositionCM() < 25 && delivery.fourbarState == Delivery.fourBarState.transfer) {
                state = autoState.obs_collect;
                built = building.notBuilt;
            }

        }else if (state == autoState.obs_collect) {

            if (built == building.notBuilt) {
                built = building.built;
                follow.setPath(paths.returnPath("colecting"));
                targetHeading = 315;
                detectingInSub = true;
                delivery.slideSetPoint(delivery.highChamberFront);
                delivery.slides = Delivery.slideState.moving;
            }

            if (follow.isFinished(1, 4) && detectingInSub){
                detectingInSub = false;
                busyDetecting = true;
                collection.sampleSorterContour.setScanning(true);
                detectingTimer.reset();
                delivery.mainPivot.setPosition(delivery.findCameraScanPosition());
            }

            if (busyDetecting){
                delivery.mainPivot.setPosition(delivery.findCameraScanPosition());
            }

            if (follow.isFinished(2, 2) && !detectingInSub && detectingTimer.milliseconds() > 2000) {
                busyDetecting = false;
            }

            if (follow.isFinished(2, 2) && collection.horizontalMotor.getCurrentPosition() < 80 && !busyDetecting && !detectingInSub){
                collection.ChamberCollect.execute();
                collection.setSlideTarget(30);
            }

            if (collection.horizontalMotor.getCurrentPosition()/(492/50)>29 && follow.isFinished(2, 2) && collection.sampleSorterContour.isScanning()){
                collection.setClawsState(Collection.clawState.drop);
                collection.sampleSorterContour.setScanning(false);
                collection.sampleMap = collection.sampleSorterContour.convertPositionsToFieldPositions(RobotPosition, delivery.getSlidePositionCM());
                collection.queueCommand(collection.autoCollectGlobal);
                collection.setChamberCollect(false);
            }

            if (follow.isFinished() && collection.horizontalMotor.getCurrentPosition()/(492/50)>2 && collection.getClawsState() == Collection.clawState.drop){

            }
        }

        if (state == autoState.finished) {
            requestOpModeStop();
        }

        odometry.queueCommand(odometry.updateLineBased);

        if (following) {
            RobotPower currentPower = follow.followPathAuto(targetHeading, odometry.Heading(), odometry.X(), odometry.Y(), odometry.getXVelocity(), odometry.getYVelocity());
            telemetry.addData("Loop time", loopTime);
            telemetry.addData("Y", odometry.Y());
            telemetry.addData("Heading", odometry.Heading());
            telemetry.addData("X", odometry.X());
            telemetry.addData("getVertical", currentPower.getVertical());
            telemetry.addData("getHorizontal", currentPower.getHorizontal());
            telemetry.addData("getPivot", currentPower.getPivot());
            telemetry.update();

            driveBase.queueCommand(driveBase.drivePowers(currentPower));
        }
    }
}
