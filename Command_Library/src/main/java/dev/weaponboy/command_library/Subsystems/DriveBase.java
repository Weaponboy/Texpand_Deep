package dev.weaponboy.command_library.Subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import dev.weaponboy.command_library.CommandLibrary.Commands.Command;
import dev.weaponboy.command_library.CommandLibrary.Commands.LambdaCommand;
import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;
import dev.weaponboy.command_library.CommandLibrary.Subsystem.SubSystem;
import dev.weaponboy.command_library.Hardware.MotorEx;
import dev.weaponboy.nexus_pathing.PathingUtility.PIDController;
import dev.weaponboy.nexus_pathing.PathingUtility.RobotPower;

public class DriveBase extends SubSystem {

    public MotorEx LF = new MotorEx();
    public MotorEx RF = new MotorEx();
    public MotorEx RB = new MotorEx();
    public MotorEx LB = new MotorEx();

    PIDController headingPID = new PIDController(0.025,0,0.0003);
    public IMU imu;
    public double strafeExtra = 1.2;

    double vertikal ;
    double turn ;
    double strafe;

    public DriveBase(OpModeEX opModeEX){
        registerSubsystem(opModeEX,driveCommand);
    }

    @Override
    public void init() {
        LF.initMotor("LF", getOpModeEX().hardwareMap);
        RF.initMotor("RF", getOpModeEX().hardwareMap);
        LB.initMotor("LB", getOpModeEX().hardwareMap);
        RB.initMotor("RB", getOpModeEX().hardwareMap);

        imu = getOpModeEX().hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection = RevHubOrientationOnRobot.LogoFacingDirection.DOWN;
        RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoFacingDirection, usbFacingDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));

        imu.resetYaw();

        LF.setDirection(DcMotorSimple.Direction.REVERSE);
        LB.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public double headindingLockMotorPower (double headingError){
        if (headingError < -180) {
            headingError = (360 + headingError);
        } else if (headingError > 180) {
            headingError = (headingError - 360);
        }
        return headingPID.calculate(headingError);
    }

    @Override
    public void execute() {
        executeEX();
    }

    public Command drivePowers (double vertical, double turn, double strafe){
        this.vertikal = -vertical;
        this.strafe = strafe;
        this.turn = turn;

        return driveCommand;
    }

    public Command drivePowers (RobotPower power){
        this.vertikal = power.getVertical();
        this.strafe = -power.getHorizontal();
        this.turn = power.getPivot();

        return driveCommand;
    }

    LambdaCommand driveCommand = new LambdaCommand(
            () -> {
            },
            () -> {
                double denominator = Math.max(1, Math.abs(vertikal)+Math.abs(strafe)+Math.abs(turn));

                LF.update((vertikal-(strafe)-turn)/denominator);
                RF.update((vertikal+(strafe)+turn)/denominator);
                LB.update((vertikal+(strafe*strafeExtra)-turn)/denominator);
                RB.update((vertikal-(strafe*strafeExtra)+turn)/denominator);

//                System.out.println("vertikal power" + vertikal);
//                System.out.println("Left front power" + LF.getPower());
            },
            () -> true
    );

    public Command driveFieldCentric(double vertical, double turn, double strafe){
        this.vertikal = -vertical;
        this.strafe = strafe;
        this.turn = turn;

        return driveField;
    }

    public void setAll(double power){
        LF.update(power);
        RF.update(power);
        LB.update(power);
        RB.update(power);
    }

    LambdaCommand driveField = new LambdaCommand(
            () -> {
            },
            () -> {
                double denominator = Math.max(1, Math.abs(vertikal)+Math.abs(strafe)+Math.abs(turn));

                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

                double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

                double rotX = vertikal * Math.cos(-heading) - strafe * Math.sin(-heading);
                double rotY = vertikal * Math.sin(-heading) + strafe * Math.cos(-heading);


                LF.update((rotX-rotY-turn)/denominator);
                RF.update((rotX+rotY+turn)/denominator);
                LB.update((rotX+rotY-turn)/denominator);
                RB.update((rotX-rotY+turn)/denominator);


            },
            () -> true


    );



}