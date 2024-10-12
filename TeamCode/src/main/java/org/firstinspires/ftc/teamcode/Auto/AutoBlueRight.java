package org.firstinspires.ftc.teamcode.Auto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;
import dev.weaponboy.command_library.Subsystems.Delivery;
import dev.weaponboy.nexus_pathing.Follower.follower;
import dev.weaponboy.nexus_pathing.PathGeneration.commands.sectionBuilder;
import dev.weaponboy.nexus_pathing.PathGeneration.pathsManager;
import dev.weaponboy.nexus_pathing.PathingUtility.RobotPower;
import dev.weaponboy.nexus_pathing.RobotUtilities.Vector2D;

@Autonomous(name = "blueRight", group = "Autos")
public class AutoBlueRight extends OpModeEX {
    double targetHeading;

    pathsManager paths = new pathsManager();
    follower follow = new follower();


    private final sectionBuilder[] rightBluePath = {
            () -> paths.addPoints(new Vector2D(339, 160), new Vector2D(254, 170))

    };

    private final sectionBuilder[] rightBlueFull = {
            () -> paths.addPoints(new Vector2D(151, 340), new Vector2D(160, 244), new Vector2D(323, 325)),
            () -> paths.addPoints(new Vector2D(55, 330), new Vector2D(160, 240))
    };
    private final sectionBuilder[] rightBlueBasket = {
            () -> paths.addPoints(new Vector2D(147, 360), new Vector2D(160, 244))
    };

    public enum autoState {
        preLoad,
        deposit,
        collect,
        humanPlayer,
    }

    public enum building {
        built,
        notBuilt
    }

    boolean following = false;

    autoState state = autoState.preLoad;
    building built = building.notBuilt;

    @Override
    public void initEX() {
        odometry.startPosition(339, 160, 180);


        paths.addNewPath("rightBluePath");
        paths.buildPath(rightBluePath);


        paths.addNewPath("rightBlueBasket");
        paths.buildPath(rightBlueBasket);


        paths.addNewPath("rightBlueFull");
        paths.buildPath(rightBlueFull);
        follow.setPath(paths.returnPath("rightBluePath"));

        delivery.queueCommand(delivery.transfer);
        delivery.queueCommand(delivery.slideSetPonts(delivery.highChamber));
    }


    @Override
    public void loopEX() {

        if (state == autoState.preLoad) {

            if (built == building.notBuilt&&delivery.slideMotor.getCurrentPosition()>350) {
                follow.setPath(paths.returnPath("rightBluePath"));
                targetHeading = 180;
                following = true;
                built = building.built;
                delivery.queueCommand(delivery.Clip);

            }

            if (follow.isFinished() && delivery.fourbarState == Delivery.fourBarState.preClip){
                delivery.queueCommand(delivery.Clip);
            }

        }

        odometry.queueCommand(odometry.updateLineBased);

        if (following){
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
