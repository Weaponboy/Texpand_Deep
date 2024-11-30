package org.firstinspires.ftc.teamcode.Testing.Auto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;
import dev.weaponboy.command_library.Subsystems.Delivery;
import dev.weaponboy.nexus_pathing.Follower.follower;
import dev.weaponboy.nexus_pathing.PathGeneration.commands.sectionBuilder;
import dev.weaponboy.nexus_pathing.PathGeneration.pathsManager;
import dev.weaponboy.nexus_pathing.PathingUtility.RobotPower;
import dev.weaponboy.nexus_pathing.RobotUtilities.Vector2D;

@Disabled
@Autonomous(name = "Blue_Right_Preload", group = "Autos")
public class Blue_Right_Preload extends OpModeEX {
    double targetHeading;


    pathsManager paths = new pathsManager();
    follower follow = new follower();


    private final sectionBuilder[] rightBluePath = {
            () -> paths.addPoints(new Vector2D(339, 160), new Vector2D(248, 170))

    };
    private final sectionBuilder[] colecting = {
            () -> paths.addPoints(new Vector2D(251, 170), new Vector2D(330,80))
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

        delivery.slideSetPonts(delivery.highChamber);
        delivery.slides = Delivery.slideState.moving;
    }


    @Override
    public void loopEX() {

        if (state == autoState.preLoad) {

            if (built == building.notBuilt && delivery.slideMotor.getCurrentPosition() > 100) {
                follow.setPath(paths.returnPath("rightBluePath"));
                targetHeading = 180;
                following = true;
                built = building.built;
                delivery.PreClip.execute();
            }

            if (!follow.isFinished() && odometry.getXVelocity() < 2 && follow.getXError() < 5){
                follow.finishPath();
            }

            if (follow.isFinished(1, 4) && !queuedClipCommands) {

                delivery.mainPivot.setPosition(delivery.findCameraScanPosition());

                for (int i = 0; i < 20; i++){

                    delivery.mainPivot.setPosition(delivery.findCameraScanPosition());
                    try {
                        Thread.sleep(100);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }

                }

                try {
                    Thread.sleep(500);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }

                queuedClipCommands = true;
                delivery.queueCommand(delivery.Clip);
                delivery.queueCommand(delivery.Clip);
                delivery.queueCommand(delivery.Clip);
                delivery.queueCommand(delivery.Clip);

            } else if (delivery.slideMotor.getCurrentPosition() < 20 && queuedClipCommands) {
                state = autoState.obs_collect;
                built = building.notBuilt;
            }

        }else if (state == autoState.obs_collect) {
            if (built == building.notBuilt) {
                follow.setPath(paths.returnPath("colecting"));
                targetHeading = 180;
            }

            if (follow.isFinished()){
                state = autoState.finished;
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

            if (follow.isFinished()){
                driveBase.queueCommand(driveBase.drivePowers(new RobotPower(0,0,0)));
            }else {
                driveBase.queueCommand(driveBase.drivePowers(currentPower));
            }

        }else {
            driveBase.queueCommand(driveBase.drivePowers(new RobotPower(0,0,0)));

        }
    }
}
