package org.firstinspires.ftc.teamcode.Auto.Red;


import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

@Autonomous(name = "Red Right", group = "Red Autos")
public class Red_Right_Full_Auto extends OpModeEX {

    double targetHeading;

    pathsManager paths = new pathsManager();
    follower follow = new follower();

    private final sectionBuilder[] preloadDelivery = {
            () -> paths.addPoints(new Vector2D(339, 160), new Vector2D(252, 178))
    };

    private final sectionBuilder[] obs_collecting = {
            () -> paths.addPoints(new Vector2D(250, 180), new Vector2D(280, 190), new Vector2D(332,122))
    };

    private final sectionBuilder[] clip_And_Collect = {
            () -> paths.addPoints(new Vector2D(332, 122), new Vector2D(280,190),new Vector2D(252, 190))
    };

    public enum autoState {
        preLoad,
        cycle_one,
        cycle_two,
        cycle_three,
        cycle_four,
        finished{
            @Override
            public autoState next() {
                return null; // see below for options for this line
            };
        };

        public autoState next() {
            // No bounds checking required here, because the last instance overrides
            return values()[ordinal() + 1];
        }
    }

    public enum CycleState{
        clip_and_collect,
        obs_collect
    }

    public enum building {
        built,
        notBuilt
    }

    boolean following = false;
    boolean collectSample = false;
    boolean clipped = false;

    autoState state = autoState.preLoad;
    autoState targetState = autoState.cycle_one;
    building built = building.notBuilt;
    building cycleBuilt = building.notBuilt;
    CycleState cycleState = CycleState.clip_and_collect;

    boolean transferring = false;
    ElapsedTime transferringWait = new ElapsedTime();

    ElapsedTime extendoWait = new ElapsedTime();

    boolean runCollect = false;

    @Override
    public void initEX() {

        delivery.setGripperState(Delivery.gripper.grab);

        delivery.griperSev.setPosition(78);

        odometry.startPosition(342.5, 164, 180);

        paths.addNewPath("preloadPath");
        paths.buildPath(preloadDelivery);

        paths.addNewPath("obs_collecting");
        paths.buildPath(obs_collecting);

        paths.addNewPath("clip_And_Collect");
        paths.buildPath(clip_And_Collect);

        follow.setPath(paths.returnPath("preloadPath"));

        delivery.queueCommand(delivery.preClip);

        FtcDashboard.getInstance().startCameraStream(collection.sampleSorterContour, 30);

        collection.sampleSorterContour.setScanning(false);
        collection.sampleSorterContour.setTargetColor(findAngleUsingContour.TargetColor.yellow);
        collection.sampleSorterContour.closestFirst = true;

        collection.setSlideTarget(15);
    }

    @Override
    public void loopEX() {

        if (state == autoState.preLoad) {

            if (built == building.notBuilt) {
                follow.setPath(paths.returnPath("preloadPath"));
                targetHeading = 180;
                following = true;
                built = building.built;
            }

            if (!follow.isFinished() && Math.abs(odometry.getXVelocity()) < 2 && follow.getXError() < 5){
                follow.finishPath();
                following = false;
            }

            if (follow.isFinished() && delivery.getCurrentCommand() != delivery.preClip && !collection.getChamberCollect()){
                delivery.queueCommand(delivery.Clip);
                //sub wall 234,5
                collection.targetPointWithExtendo(new Vector2D(202, 190));
            }

            if (Math.abs(collection.getSlidePositionCM() - collection.getSlideTarget()) < 1 && follow.isFinished() && !collection.getChamberCollect()){
                collection.queueCommand(collection.collect);
                collection.setChamberCollect(true);
                extendoWait.reset();
            }

            if (collection.getChamberCollect() && extendoWait.milliseconds() > 1000 && !runCollect){
                collection.queueCommand(collection.collect);
                runCollect = true;
            }

            if (follow.isFinished() && delivery.getSlidePositionCM() < 5 && collection.getSlidePositionCM() < 2 && delivery.getCurrentCommand() != delivery.preClip && collection.getChamberCollect() && collection.clawSensor.isPressed()) {
                state = autoState.cycle_one;
                built = building.notBuilt;
                cycleBuilt = building.notBuilt;
            }

        }else {

            if (built == building.notBuilt){
                cycleState = CycleState.obs_collect;
                built = building.built;
            }

            fullCycle();

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
        }else {
            driveBase.queueCommand(driveBase.drivePowers(new RobotPower(0, 0,follow.getTurnPower(targetHeading, odometry.Heading()))));
        }
    }

    public void fullCycle(){

        if (cycleState == CycleState.obs_collect) {

            if (cycleBuilt == building.notBuilt) {
                cycleBuilt = building.built;
                follow.setPath(paths.returnPath("obs_collecting"));
                following = true;
                targetHeading = 270;
            }

            if (odometry.Heading() > 240 && !follow.isFinished(2,2) && !collectSample){
                collection.targetPointWithExtendo(new Vector2D(341, 70));
                collection.griperRotate.setPosition(180);
            }

            if (follow.isFinished(2,2) && !collectSample && collection.horizontalMotor.getVelocity() < 10){
                following = false;
                collection.queueCommand(collection.obs_Collect);
                collectSample = true;
            } else if (follow.isFinished(2,2) && collectSample && collection.getSlideTarget() == 0){
                cycleState = CycleState.clip_and_collect;
                cycleBuilt = building.notBuilt;
            }

        }else if (cycleState == CycleState.clip_and_collect) {

            if (cycleBuilt == building.notBuilt) {
                cycleBuilt = building.built;
                follow.setPath(paths.returnPath("clip_And_Collect"));
                following = true;
                clipped = false;
                runCollect = false;

                targetHeading = 180;
            }

            if (!follow.isFinished() && Math.abs(odometry.getXVelocity()) < 2 && follow.getXError() < 5){
                following = false;
                follow.finishPath();
            }

            if (collection.clawSensor.isPressed() && !transferring && collection.slidesReset.isPressed()){
                delivery.queueCommand(delivery.transfer);
                transferring = true;
                transferringWait.reset();
            } else if (transferring && transferringWait.milliseconds() > 500 && transferringWait.milliseconds() < 600) {
                collection.setClawsState(Collection.clawState.drop);
            } else if (transferring && transferringWait.milliseconds() > 800) {
                transferring = false;
                delivery.queueCommand(delivery.preClip);
            }

            if (follow.isFinished() && delivery.getCurrentCommand() != delivery.preClip && !transferring){
                delivery.queueCommand(delivery.Clip);
                clipped = true;
            }

            if (state == targetState){

                if (follow.isFinished(2, 2) && clipped && delivery.getCurrentCommand() != delivery.preClip && delivery.getSlidePositionCM() < 4) {
                    state = autoState.finished;
                }

            }else {

                if (follow.isFinished() && delivery.getCurrentCommand() != delivery.preClip && !collection.getChamberCollect()){
                    delivery.queueCommand(delivery.Clip);
                    collection.targetPointWithExtendo(new Vector2D(202, 190));
                }

                if (Math.abs(collection.getSlidePositionCM() - collection.getSlideTarget()) < 1 && follow.isFinished() && !collection.getChamberCollect()){
                    collection.queueCommand(collection.collect);
                    collection.setChamberCollect(true);
                    extendoWait.reset();
                }

                if (collection.getChamberCollect() && extendoWait.milliseconds() > 1000 && !runCollect){
                    collection.queueCommand(collection.collect);
                    runCollect = true;
                }

                if (follow.isFinished(2, 2) && clipped && delivery.getCurrentCommand() != delivery.preClip && delivery.getSlidePositionCM() < 4) {
                    state.next();
                    built = building.notBuilt;
                    cycleBuilt = building.notBuilt;
                }

            }

        }

    }
}
