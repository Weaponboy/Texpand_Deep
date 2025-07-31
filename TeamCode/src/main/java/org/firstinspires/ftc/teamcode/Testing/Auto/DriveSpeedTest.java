package org.firstinspires.ftc.teamcode.Testing.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;
import dev.weaponboy.nexus_pathing.Follower.follower;
import dev.weaponboy.nexus_pathing.PathGeneration.commands.sectionBuilder;
import dev.weaponboy.nexus_pathing.PathGeneration.pathsManager;
import dev.weaponboy.nexus_pathing.PathingUtility.RobotPower;
import dev.weaponboy.nexus_pathing.RobotUtilities.Vector2D;


@Autonomous(name = "Drive_Speed_Test", group = "Testing")
public class DriveSpeedTest extends OpModeEX {

    double targetHeading;
    pathsManager paths = new pathsManager();
    follower follow = new follower();

    double initialVelocity = 0;
    double finalVelocity = 0;
    ElapsedTime accelTimer = new ElapsedTime();

    private final sectionBuilder[] straightLine = {
            () -> paths.addPoints(new Vector2D(0, 0), new Vector2D(150, 0))
    };

    @Override
    public void initEX() {
        odometry.startPosition(0,0,0);

        paths.addNewPath("line");
        paths.buildPath(straightLine, 225);

        follow.setPath(paths.returnPath("line"));
    }

    @Override
    public void loopEX() {
        RobotPower currentPower = follow.followPathAuto(targetHeading, odometry.Heading(), odometry.X(), odometry.Y(), odometry.getXVelocity(), odometry.getYVelocity());
        driveBase.queueCommand(driveBase.drivePowers(currentPower));

        if (odometry.X() > 90 && initialVelocity == 0){
            initialVelocity = Math.abs(odometry.getXVelocity()) + Math.abs(odometry.getYVelocity());
            System.out.println("Initial velocity" + initialVelocity);
            accelTimer.reset();
        }

        if (odometry.X() > 150 && finalVelocity == 0){
            finalVelocity = Math.abs(odometry.getXVelocity()) + Math.abs(odometry.getYVelocity());

            double accel = (finalVelocity - initialVelocity) / accelTimer.seconds();
            System.out.println("accel" + accel);
            System.out.println("Final Velocity" + finalVelocity);
        }

        System.out.println("Robot velocity" + Math.abs(odometry.getXVelocity()) + Math.abs(odometry.getYVelocity()));
        System.out.println("X position" + odometry.X());
//        System.out.println("X Power" + currentPower.getVertical());
    }
}
