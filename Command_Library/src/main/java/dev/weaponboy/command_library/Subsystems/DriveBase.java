package dev.weaponboy.command_library.Subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import dev.weaponboy.command_library.CommandLibrary.Commands.Command;
import dev.weaponboy.command_library.CommandLibrary.Commands.LambdaCommand;
import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;
import dev.weaponboy.command_library.CommandLibrary.Subsystem.SubSystem;

public class DriveBase extends SubSystem {
    DcMotorEx leftFD;
    DcMotorEx rightFD;
    DcMotorEx backRD;
    DcMotorEx backLD;
    public IMU imu;

    double vertikal ;
    double turn ;
    double strafe;
    public DriveBase(OpModeEX opModeEX){

        registerSubsystem(opModeEX,driveCommand);
    }

    @Override
    public void init() {
        leftFD = getOpModeEX().hardwareMap.get(DcMotorEx.class, "leftFD");
        rightFD = getOpModeEX().hardwareMap.get(DcMotorEx.class, "rightFD");
        backLD = getOpModeEX().hardwareMap.get(DcMotorEx.class, "backLD");
        backRD = getOpModeEX().hardwareMap.get(DcMotorEx.class, "backRD");

        imu = getOpModeEX().hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoFacingDirection, usbFacingDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));


        leftFD.setDirection(DcMotorSimple.Direction.REVERSE);
        backLD.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void execute() {
        executeEX();
    }

    public Command drivePowers (double vertikal, double turn, double strafe){
        this.vertikal =vertikal;
        this.strafe =strafe;
        this.turn =turn*0.5;

        return driveCommand;
    }
    LambdaCommand driveCommand = new LambdaCommand(
            () -> {
                System.out.println("init");
            },
            () -> {
                double denominator = Math.max(1, Math.abs(vertikal)+Math.abs(strafe)+Math.abs(turn));

                leftFD.setPower((vertikal-strafe-turn)/denominator);
                rightFD.setPower((vertikal+strafe+turn)/denominator);
                backLD.setPower((vertikal+strafe-turn)/denominator);
                backRD.setPower((vertikal-strafe+turn)/denominator);
            },
            () -> true


    );

    public Command driveFieldCentric(double vertikal, double turn, double strafe){
        this.vertikal =vertikal;
        this.strafe =strafe;
        this.turn =turn*0.5;

        return driveField;
    }
    LambdaCommand driveField = new LambdaCommand(
            () -> {
                System.out.println("init");
            },
            () -> {
                double denominator = Math.max(1, Math.abs(vertikal)+Math.abs(strafe)+Math.abs(turn));
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                 double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

                 double rotX = vertikal * Math.cos(-heading) - strafe * Math.sin(-heading);
                 double rotY = vertikal * Math.sin(-heading) + strafe * Math.cos(-heading);


                 leftFD.setPower((rotX-rotY-turn)/denominator);
                rightFD.setPower((rotX+rotY+turn)/denominator);
                backLD.setPower((rotX+rotY-turn)/denominator);
                backRD.setPower((rotX-rotY+turn)/denominator);


            },
            () -> true


    );



}