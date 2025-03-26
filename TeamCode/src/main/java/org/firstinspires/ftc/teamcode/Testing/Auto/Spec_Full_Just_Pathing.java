package org.firstinspires.ftc.teamcode.Testing.Auto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;
import dev.weaponboy.command_library.Subsystems.Collection;
import dev.weaponboy.command_library.Subsystems.Delivery;
import dev.weaponboy.nexus_pathing.Follower.follower;
import dev.weaponboy.nexus_pathing.PathGeneration.commands.sectionBuilder;
import dev.weaponboy.nexus_pathing.PathGeneration.pathsManager;
import dev.weaponboy.nexus_pathing.PathingUtility.PathingPower;
import dev.weaponboy.nexus_pathing.PathingUtility.RobotPower;
import dev.weaponboy.nexus_pathing.RobotUtilities.Vector2D;

@Disabled
@Autonomous(name = "Spec_Full_Just_Pathing", group = "AA Comp Autos")
public class Spec_Full_Just_Pathing extends OpModeEX {

    double targetHeading;

    pathsManager paths = new pathsManager();
    follower follow = new follower();

    double adjustedTarget = 0;
    Vector2D powerPID = new Vector2D();

    private final sectionBuilder[] preloadDelivery = {
            () -> paths.addPoints(new Vector2D(339, 160), new Vector2D(252, 175))
    };

    private final sectionBuilder[] obs_collecting = {
            () -> paths.addPoints(new Vector2D(250, 180), new Vector2D(280, 190), new Vector2D(332,122))
    };

    private final sectionBuilder[] clip_And_Collect_One = {
            () -> paths.addPoints(new Vector2D(332, 122), new Vector2D(280,190),new Vector2D(250, 182))
    };

    private final sectionBuilder[] clip_And_Collect_Two = {
            () -> paths.addPoints(new Vector2D(332, 122), new Vector2D(280,190),new Vector2D(250, 174))
    };

    private final sectionBuilder[] clip_And_Collect_Three = {
            () -> paths.addPoints(new Vector2D(332, 122), new Vector2D(280,190),new Vector2D(250, 167))
    };

    private final sectionBuilder[] spikeMarks = {
            () -> paths.addPoints(new Vector2D(252, 178), new Vector2D(290,172),new Vector2D(290, 106))
    };

    private final sectionBuilder[] spikeMarks2 = {
            () -> paths.addPoints(new Vector2D(252, 178), new Vector2D(310,172),new Vector2D(290, 95))
    };

    private final sectionBuilder[] spikeMarks3 = {
            () -> paths.addPoints(new Vector2D(252, 178), new Vector2D(310,172),new Vector2D(286, 64))
    };

    private final sectionBuilder[] goToGrabPreload = {
            () -> paths.addPoints(new Vector2D(290, 68), new Vector2D(290,96),new Vector2D(294, 126))
    };

    private final sectionBuilder[] goToGrab = {
            () -> paths.addPoints(new Vector2D(260, 165), new Vector2D(280,156),new Vector2D(295, 125))
    };

    private final sectionBuilder[] goToDrop = {
            () -> paths.addPoints(new Vector2D(292, 129), new Vector2D(282,145),new Vector2D(260, 160))
    };

    public enum autoState {
        preLoad,
        spike_one,
        spike_two,
        spike_three,
        cycle_one,
        cycle_two,
        cycle_three,
        cycle_four,
        finished,
        cycle_five,
        cycle_six,
        final_Clip;


        public static autoState next(autoState current) {
            autoState[] values = autoState.values();
            int nextIndex = (current.ordinal() + 1) % values.length; // Wrap around to the first enum
            return values[nextIndex];
        }
    }
    public enum targetExtendo{
        notSet,
        one,
        two,
        three,
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
    boolean headingAdjustment = false;
    boolean headingOverride = true;

    autoState state = autoState.preLoad;
    autoState targetState = autoState.cycle_four;
    building built = building.notBuilt;
    building cycleBuilt = building.notBuilt;
    CycleState cycleState = CycleState.clip_and_collect;
    targetExtendo TargetExtendo = targetExtendo.notSet;

    boolean transferring = false;
    ElapsedTime transferringWait = new ElapsedTime();

    ElapsedTime extendoWait = new ElapsedTime();

    boolean runCollect = false;

    boolean PIDToPoint = false;
    boolean firstSpike = false;

    @Override
    public void initEX() {

        delivery.setGripperState(Delivery.gripper.grab);

        odometry.startPosition(342.5, 164, 180);

        paths.addNewPath("preloadPath");
        paths.buildPath(preloadDelivery);

        paths.addNewPath("obs_collecting");
        paths.buildPath(obs_collecting);

        paths.addNewPath("clip_One");
        paths.buildPath(clip_And_Collect_One);

        paths.addNewPath("clip_Two");
        paths.buildPath(clip_And_Collect_Two);

        paths.addNewPath("clip_Three");
        paths.buildPath(clip_And_Collect_Three);

        paths.addNewPath("spike");
        paths.buildPath(spikeMarks);

        paths.addNewPath("spike2");
        paths.buildPath(spikeMarks2);

        paths.addNewPath("spike3");
        paths.buildPath(spikeMarks3);

        paths.addNewPath("goToGrabPreload");
        paths.buildPath(goToGrabPreload);

        paths.addNewPath("goToGrap");
        paths.buildPath(goToGrab);

        paths.addNewPath("goToDrob");
        paths.buildPath(goToDrop);

        follow.setPath(paths.returnPath("preloadPath"));

    }

    @Override
    public void loopEX() {

        if (state == autoState.preLoad) {

            if (built == building.notBuilt) {
                follow.setPath(paths.returnPath("preloadPath"));
                targetHeading = 180;
                following = true;
                clipped = false;
                runCollect = false;
                built = building.built;

                delivery.slideSetPoint(30);
                delivery.slides = Delivery.slideState.moving;

//                delivery.queueCommand(delivery.preClipFront);
            }

            if (!follow.isFinished() && Math.abs(odometry.getXVelocity()) < 2 && follow.getXError() < 5){
                follow.finishPath();
                following = false;
                PIDToPoint = false;
            }

            if (follow.isFinished()) {
                state = autoState.spike_one;
                built = building.notBuilt;
                runCollect = false;
            }

        }else if (state == autoState.spike_one){

            if (built == building.notBuilt) {
                follow.setPath(paths.returnPath("spike"));
                targetHeading = 225;
                following = true;
                built = building.built;

                TargetExtendo = targetExtendo.notSet;
            }

            if (follow.isFinished(4,4) && targetHeading != 300){
                targetHeading = 300;

                following = true;
            }

            if (odometry.Heading() > 280){
                state = autoState.spike_two;
                built = building.notBuilt;
                follow.setExtendoHeading(false);
            }

        } else if (state == autoState.spike_two){

            if (built == building.notBuilt) {
                follow.setPath(paths.returnPath("spike2"));
                targetHeading = 235;
                following = true;
                built = building.built;
                TargetExtendo = targetExtendo.notSet;
            }


            if (follow.isFinished(4,4) && targetHeading != 292){
                follow.setExtendoHeading(true);

                targetHeading = 292;
            }

            if (odometry.Heading() > 272){
                state = autoState.spike_three;
                built = building.notBuilt;
                follow.setExtendoHeading(false);
            }

        } else if (state == autoState.spike_three){

            if (built == building.notBuilt) {
                follow.setPath(paths.returnPath("spike3"));
                targetHeading = 235;
                following = true;
                built = building.built;
                TargetExtendo = targetExtendo.notSet;
            }

            if (follow.isFinished(4,4) && targetHeading != 292){
                follow.setExtendoHeading(true);
                targetHeading = 292;
            }

            if (odometry.Heading() > 272){
                state = autoState.cycle_one;
                built = building.notBuilt;
                runCollect = false;
                cycleBuilt = building.notBuilt;
                follow.setExtendoHeading(true);
            }

        }else {

            if (built == building.notBuilt) {

                cycleBuilt = building.notBuilt;

                cycleState = CycleState.obs_collect;

                built = building.built;

                System.out.println("ran cycle build: " + state.name());

            }

            fullCycle();

        }

        if (state == autoState.finished) {
            requestOpModeStop();
        }

        odometry.queueCommand(odometry.updateLineBased);

        if (following) {
            RobotPower currentPower = follow.followPathAuto(targetHeading, odometry.Heading(), odometry.X(), odometry.Y(), odometry.getXVelocity(), odometry.getYVelocity());
//            telemetry.addData("Loop time", loopTime);
//            telemetry.addData("Y", odometry.Y());
//            telemetry.addData("Heading", odometry.Heading());
//            telemetry.addData("X", odometry.X());
//            telemetry.addData("getVertical", currentPower.getVertical());
//            telemetry.addData("getHorizontal", currentPower.getHorizontal());
//            telemetry.addData("getPivot", currentPower.getPivot());
//            telemetry.update();

            driveBase.queueCommand(driveBase.drivePowers(currentPower));
        }else {
            if (!headingOverride) {
                if (Math.abs(targetHeading - odometry.Heading()) > 5) {
                    headingAdjustment = true;
                } else {
                    headingAdjustment = false;
                }
            } else {
                headingAdjustment = false;
            }

            if (headingAdjustment) {
                double error = targetHeading - odometry.Heading();

                if (Math.abs(odometry.getXVelocity()) < 3 && Math.abs(odometry.getYVelocity()) < 3) {
                    if (error > 0) {
                        adjustedTarget += 0.4;
                    } else {
                        adjustedTarget -= 0.4;
                    }
                } else {
                    adjustedTarget = 0;
                }

                driveBase.queueCommand(driveBase.drivePowers(new RobotPower(powerPID.getX(), powerPID.getY(), follow.getTurnPower(targetHeading + adjustedTarget, odometry.Heading(), odometry.getXVelocity(), odometry.getYVelocity()))));
            } else {
                driveBase.queueCommand(driveBase.drivePowers(new RobotPower(powerPID.getX(), powerPID.getY(), 0)));
            }
        }

        telemetry.addData("Y", odometry.Y());
        telemetry.addData("Heading", odometry.Heading());
        telemetry.addData("X", odometry.X());

        telemetry.addData("PID.X", powerPID.getX());
        telemetry.addData("PID.Y", powerPID.getY());
        telemetry.addData("follow.isFinished", follow.isFinished());
        telemetry.addData("follow.isFinished", follow.isFinished());
        telemetry.addData("targetHeading", targetHeading);
        telemetry.addData("PIDToPoint", PIDToPoint);
        telemetry.addData("following", following);
        telemetry.update();
    }

    public void fullCycle(){

        Vector2D targetExtendoPoint = new Vector2D(356, 91);

        if (cycleState == CycleState.obs_collect) {

            if (cycleBuilt == building.notBuilt) {
                cycleBuilt = building.built;

                following = false;
                collectSample = false;
                headingOverride = false;
                PIDToPoint = true;
                targetHeading = 325;
            }

            if (PIDToPoint) {
                PathingPower power = follow.pidToPoint(new Vector2D(odometry.X(), odometry.Y()), new Vector2D(290, 141), odometry.Heading(), odometry.getXVelocity(), odometry.getYVelocity());
                powerPID = new Vector2D(power.getVertical(), power.getHorizontal());
            } else {
                powerPID = new Vector2D();
            }

            if (Math.abs(powerPID.getX()) < 0.1 && Math.abs(powerPID.getY()) < 0.1){
                cycleState = CycleState.clip_and_collect;
                cycleBuilt = building.notBuilt;
                System.out.println("moved to delivery: " + state.name());
            }

        }else if (cycleState == CycleState.clip_and_collect) {

            if (cycleBuilt == building.notBuilt) {

                cycleBuilt = building.built;
                powerPID = new Vector2D();
                follow.setPath(paths.returnPath("goToDrob"));
                following = true;
                clipped = false;

                PIDToPoint = false;

                targetHeading = 345;
            }

            if (!follow.isFinished() && Math.abs(odometry.getXVelocity()) < 2 && follow.getXError() < 5){
                following = false;
                follow.finishPath();
            }

            if (state == targetState){
                if (follow.isFinished(2, 2)) {
                    state = autoState.finished;
                    built = building.notBuilt;
                    cycleBuilt = building.notBuilt;
                    System.out.println("finished: " + state.name());
                }
            }else{
                if (follow.isFinished(2, 2)) {
                    state = autoState.next(state);
                    built = building.notBuilt;
                    cycleBuilt = building.notBuilt;
                    follow.setExtendoHeading(false);
                    System.out.println("incremented state: " + state.name());
                    System.out.println("Y" + odometry.Y());
                    System.out.println("Heading" + odometry.Heading());
                    System.out.println("X" + odometry.X());
                }
            }

        }

    }
}
