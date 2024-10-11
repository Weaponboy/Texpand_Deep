package org.firstinspires.ftc.teamcode.Testing.Drivetrain;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;
import dev.weaponboy.command_library.Subsystems.Odometry;

@TeleOp
//@Disabled
public class TestMaxVelocity extends OpModeEX {

    ElapsedTime elapsedTimeX = new ElapsedTime();
    ElapsedTime elapsedTimeY = new ElapsedTime();

    double maxVecticalVelo;
    double maxHorzontalVelo;

    double maxVecticalAcc;
    double maxHorzontalAcc;

    double lastTimeX;
    double lastTimeY;

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

    }

    @Override
    public void loopEX() {

        driveBase.queueCommand(driveBase.drivePowers(gamepad1.right_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x));

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

        telemetry.addData("x velo", maxVecticalVelo);
        telemetry.addData("y velo", maxHorzontalVelo);
        telemetry.addData("lastTime", lastTimeX);
        telemetry.addData("lastTime", lastTimeY);

        telemetry.addData("x velo actual", odometry.getXVelocity());
        telemetry.addData("y velo actual", odometry.getYVelocity());
        telemetry.update();

    }
}
