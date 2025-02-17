package org.firstinspires.ftc.teamcode.Testing.Drivetrain;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;
import dev.weaponboy.command_library.Subsystems.Odometry;
import dev.weaponboy.nexus_pathing.Follower.follower;
import dev.weaponboy.nexus_pathing.PathGeneration.commands.sectionBuilder;
import dev.weaponboy.nexus_pathing.PathGeneration.pathsManager;
import dev.weaponboy.nexus_pathing.PathingUtility.RobotPower;
import dev.weaponboy.nexus_pathing.RobotUtilities.Vector2D;

@TeleOp
public class TestMaxVelocity extends OpModeEX {

    ElapsedTime elapsedTimeX = new ElapsedTime();
    ElapsedTime elapsedTimeY = new ElapsedTime();

    pathsManager paths = new pathsManager();
    boolean following = false;
    follower follow = new follower();
    double targetHeading;


    double maxVecticalVelo;
    double maxHorzontalVelo;

    double maxVecticalAcc;
    double maxHorzontalAcc;

    double lastTimeX;
    double lastTimeY;

    double turnPower = 0;
    double horizontal = 0;
    double lastHeading;
    private final sectionBuilder[] drive = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(0, 0), new Vector2D(20, 0), new Vector2D(45, 0)),
    };

//    public void getAcc(){
//
//        drive.RF.setPower(1);
//        drive.LF.setPower(1);
//        drive.RB.setPower(1);
//        drive.LB.setPower(1);
//
//        elapsedTimeX.reset();
//
//        while (odo.X < 80){
//            odo.update();
//
//            drive.RF.setPower(1);
//            drive.LF.setPower(-1);
//            drive.RB.setPower(1);
//            drive.LB.setPower(-1);
//
//            maxVecticalAcc = odo.getVerticalVelocity();
//        }
//
//        drive.RF.setPower(0);
//        drive.LF.setPower(0);
//        drive.RB.setPower(0);
//        drive.LB.setPower(0);
//
//        maxVecticalAcc = odo.getVerticalVelocity();
//
//        lastTimeX = elapsedTimeX.seconds();
//
//    }

    @Override
    public void initEX() {
        odometry.startPosition(0, 0, 180);
        paths.addNewPath("drive");
        paths.buildPath(drive);


    }

    @Override
    public void loopEX() {
        if (gamepad2.a){
            targetHeading = 0;
            following = true;
            follow.setPath(paths.returnPath("drive"));
        }

        horizontal = -gamepad2.right_stick_x*0.5;
        double lastHor = -lastGamepad2.right_stick_x*0.5;

        if (gamepad2.left_stick_x == 0 && Math.abs(horizontal) > 0){
            if (Math.abs(horizontal) > 0 && lastHor == 0){
                lastHeading = odometry.Heading();
            }
            turnPower = -driveBase.headindingLockMotorPower(lastHeading - odometry.Heading());
        }else {
            turnPower = -gamepad2.left_stick_x*0.5;
        }

        // drive base code
        driveBase.queueCommand(driveBase.drivePowers(-gamepad2.right_stick_y, turnPower, -gamepad2.right_stick_x));


        double differenceX = odometry.getXVelocity()- maxVecticalVelo;

        if (differenceX > 0){
            maxVecticalVelo = odometry.getXVelocity();
        }

        if (odometry.getXVelocity() < 5 && maxVecticalVelo < 190){
            elapsedTimeX.reset();
        }

        if (maxVecticalVelo > 190 && lastTimeX < 1){
            lastTimeX = elapsedTimeX.seconds();
        }

        double differenceY = odometry.getYVelocity() - maxHorzontalVelo;

        if (differenceY > 0){
            maxHorzontalVelo = odometry.getYVelocity();
        }

        if (odometry.getYVelocity() < 5 && maxHorzontalVelo < 190){
            elapsedTimeY.reset();
        }

        if (maxHorzontalVelo > 130 && lastTimeY < 1){
            lastTimeY = elapsedTimeY.seconds();
        }
        if (following) {
            RobotPower currentPower = follow.followPathAuto(targetHeading, odometry.Heading(), odometry.X(), odometry.Y(), odometry.getXVelocity(), odometry.getYVelocity());
//
            driveBase.queueCommand(driveBase.drivePowers(currentPower));
        }

        telemetry.addData("x velo", maxVecticalVelo);
        telemetry.addData("y velo", maxHorzontalVelo);
        telemetry.addData("lastTime", lastTimeX);
        telemetry.addData("lastTime", lastTimeY);

        telemetry.addData("x velo actual", odometry.getXVelocity());
        telemetry.addData("y velo actual", odometry.getYVelocity());
        telemetry.addData("power",driveBase.RB.getPower());
        telemetry.update();

    }
}
