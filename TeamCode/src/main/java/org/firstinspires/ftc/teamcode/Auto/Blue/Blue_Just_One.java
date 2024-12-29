package org.firstinspires.ftc.teamcode.Auto.Blue;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.opencv.core.Point;

import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;
import dev.weaponboy.command_library.Subsystems.Collection;
import dev.weaponboy.command_library.Subsystems.Delivery;
import dev.weaponboy.nexus_pathing.Follower.follower;
import dev.weaponboy.nexus_pathing.PathGeneration.commands.sectionBuilder;
import dev.weaponboy.nexus_pathing.PathGeneration.pathsManager;
import dev.weaponboy.nexus_pathing.PathingUtility.RobotPower;
import dev.weaponboy.nexus_pathing.RobotUtilities.Vector2D;

@Autonomous
public class Blue_Just_One extends OpModeEX {

    pathsManager paths = new pathsManager();

    follower follow = new follower();

    double targetHeading;

    enum AutoState {
        preload,
        spikeCollect,
        spikeDepo,
        finished
    }

    public enum building{
        built,
        notBuilt
    }

    boolean pathing = false;
    boolean autoQueued = false;
    boolean pullDownSlides = false;
    boolean drop = false;

    public building built = building.notBuilt;
    public AutoState state = AutoState.preload;

    private final sectionBuilder[] preloadPath = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(344.3, 275), new Vector2D(322, 282), new Vector2D(332, 332)),
    };

    private final sectionBuilder[] spikeOne = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(334, 334), new Vector2D(311, 326)),
    };

    private final sectionBuilder[] subDeposit = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(200, 232), new Vector2D(204, 288), new Vector2D(331, 331)),
    };

    @Override
    public void initEX() {
        paths.addNewPath("preloadPath");
        paths.buildPath(preloadPath);

        paths.addNewPath("spikeOne");
        paths.buildPath(spikeOne);

        follow.setPath(paths.returnPath("dropBasket"));
        paths.buildPath(subDeposit);

        FtcDashboard.getInstance().startCameraStream(collection.sampleSorterContour, 30);

        collection.sampleSorterContour.closestFirst = true;
    }

    @Override
    public void loopEX() {
        if (state == AutoState.preload) {

            if (built == building.notBuilt) {

                delivery.slideSetPoint(delivery.autoHighBasket);
                delivery.slides = Delivery.slideState.moving;

                follow.setPath(paths.returnPath("preloadPath"));
                targetHeading = 235;
                built = building.built;
                drop = true;


                pathing = true;
                follow.setExtendoHeading(true);
            }

            if (delivery.slideMotor.getCurrentPosition() > 200 && delivery.fourbarState == Delivery.fourBarState.transfer){
                delivery.queueCommand(delivery.deposit);
            }

            if (delivery.fourbarState == Delivery.fourBarState.basketDeposit && drop && delivery.getSlidePositionCM() > 52 - 4 && follow.isFinished(12,12)) {
                delivery.queueCommand(delivery.deposit);

                pathing = false;

                drop = false;
            }

            if (follow.isFinished(13, 13) && delivery.fourbarState == Delivery.fourBarState.basketDeposit && delivery.getGripperState() == Delivery.gripper.drop) {
                state = AutoState.spikeCollect;
                collection.setSlideTarget(20);
                built = building.notBuilt;

            }
        }else if (state == AutoState.spikeCollect){

            if (built == building.notBuilt){

                follow.setPath(paths.returnPath("spikeOne"));

                targetHeading = 203;

                built = building.notBuilt;

                collection.queueCommand(collection.collect);

                pathing = true;
                pullDownSlides = false;
                autoQueued = false;
                follow.setExtendoHeading(false);
            }

            if (odometry.X() < 320 && !pullDownSlides){
                pullDownSlides = true;
                delivery.queueCommand(delivery.deposit);
            }

            if (!autoQueued){
                collection.targetPointWithExtendoNoArm(new Vector2D(250,306));
            }

            Vector2D armPosition = collection.extendoPoint();

            if (Math.abs(armPosition.getX() - 249) < 20 && collection.horizontalMotor.getVelocity() < 5 && !autoQueued && collection.getFourBarState() == Collection.fourBar.preCollect && Math.abs(odometry.Heading() - targetHeading) < 5){

                autoQueued = true;

                pathing = false;

                collection.queueCommand(collection.extendoTargetPoint(new Point(250, 306)));

                collection.queueCommand(collection.collect);

                collection.queueCommand(collection.transfer);

                collection.queueCommand(collection.transferDrop);

                collection.queueCommand(delivery.closeGripper);

                collection.queueCommand(collection.openGripper);

            }

            if (collection.getClawsState() == Collection.clawState.grab && collection.fourBarMainPivot.getPositionDegrees() > 140){
                state = AutoState.spikeDepo;
                built = building.notBuilt;
            }

        } else if (state == AutoState.spikeDepo) {

            if (built == building.notBuilt){

                follow.setPath(paths.returnPath("dropBasket"));

                targetHeading = 225;

                built = building.notBuilt;
                pathing = true;
                drop = true;
                follow.setExtendoHeading(true);
            }

            if (collection.getCurrentCommand() == collection.defaultCommand){
                delivery.slideSetPoint(delivery.autoHighBasket);
                delivery.slides = Delivery.slideState.moving;
            }

            if (delivery.slideMotor.getCurrentPosition() > 695 && delivery.fourbarState == Delivery.fourBarState.transfer){
                delivery.queueCommand(delivery.deposit);
            }

            if (follow.isFinished(8,8) && delivery.fourbarState == Delivery.fourBarState.basketDeposit && delivery.getGripperState() == Delivery.gripper.grab){
                state = AutoState.finished;
            }

        }

        if (state== AutoState.finished){
            requestOpModeStop();
        }

        if (pathing){

            odometry.queueCommand(odometry.updateLineBased);
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
        }else{
            driveBase.queueCommand(driveBase.drivePowers(new RobotPower(0,0, 0)));
        }
    }
}
