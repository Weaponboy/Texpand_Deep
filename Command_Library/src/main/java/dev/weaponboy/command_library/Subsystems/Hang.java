package dev.weaponboy.command_library.Subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.weaponboy.command_library.CommandLibrary.Commands.LambdaCommand;
import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;
import dev.weaponboy.command_library.CommandLibrary.Subsystem.SubSystem;
import dev.weaponboy.command_library.Hardware.MotorEx;
import dev.weaponboy.command_library.Hardware.ServoDegrees;
import dev.weaponboy.nexus_pathing.PathingUtility.PIDController;

public class Hang extends SubSystem {

    public IMU imu;

    public Servo hang1;
    public Servo hang2;

    boolean servoActive = false;
    boolean engage = false;

    ElapsedTime engageTime = new ElapsedTime();

    double targetTilt = -17.4;

    public ServoDegrees PTO = new ServoDegrees();

    public MotorEx hangPower = new MotorEx();
    public MotorEx hangPower2 = new MotorEx();
    public MotorEx hangPower3 = new MotorEx();

    PIDController adjustment = new PIDController(0.15, 0, 0.05);

    public Hang(OpModeEX opModeEX){
        registerSubsystem(opModeEX, stop);
    }

    @Override
    public void init() {
        imu = getOpModeEX().hardwareMap.get(IMU.class, "imu");

        hang1 = getOpModeEX().hardwareMap.get(Servo.class,"hang1");
        hang2 = getOpModeEX().hardwareMap.get(Servo.class,"hang2");

        hangPower2.initMotor("slideMotor",getOpModeEX().hardwareMap);
        hangPower2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangPower2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hangPower3.initMotor("slideMotor2",getOpModeEX().hardwareMap);
        hangPower3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangPower3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        hang1.setDirection(Servo.Direction.REVERSE);

        RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection = RevHubOrientationOnRobot.LogoFacingDirection.DOWN;
        RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoFacingDirection, usbFacingDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));

        imu.resetYaw();

        hang1.setPosition(0.5);
        hang2.setPosition(0.5);

        PTO.initServo("hangPTO",getOpModeEX().hardwareMap);
        PTO.setRange(new PwmControl.PwmRange(500, 2500), 270);
//        gripServo.setRange(180);
//        griperRotate.setRange(new PwmControl.PwmRange(500, 2500), 180);
//
        hangPower.initMotor("hangPower",getOpModeEX().hardwareMap);
        hangPower.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        PTO.setDirection(Servo.Direction.REVERSE);

        hang1.setPosition(0.5);
        hang2.setPosition(0.5);

        hangPower2.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public boolean getServoActive() {
        return servoActive;
    }

    public void setServoActive(boolean servoActive) {
        this.servoActive = servoActive;
    }

    public void pullUp(double SlidePosition){
        hangPower.update(-0.4 + adjustment.calculate(2, SlidePosition));
    }

    @Override
    public void execute() {
        executeEX();
        if (servoActive){
            if (imu.getRobotYawPitchRollAngles().getPitch() > targetTilt){
                hang1.setPosition(0);
                hang2.setPosition(0);
            }
            if (imu.getRobotYawPitchRollAngles().getPitch() < targetTilt){
                hang2.setPosition(0.5);
                hang1.setPosition(0.5);
            }

        }
    }

    public LambdaCommand stop = new LambdaCommand(
            () -> {},
            () -> {
//                hang2.setPosition(0.5);
//                hang1.setPosition(0.5);

            },
            () -> true
    );

    public LambdaCommand deployArms  = new LambdaCommand(
            () -> {
                engageTime.reset();
            },
            () -> {
                hang2.setPosition(1);
                hang1.setPosition(1);

                if (engageTime.milliseconds() > 350){
                    hang2.setPosition(0.5);
                    hang1.setPosition(0.5);
                }
            },
            () -> engageTime.milliseconds() > 400
    );

    public LambdaCommand Engage  = new LambdaCommand(
            () -> {
                engageTime.reset();
            },
            () -> {
                PTO.setPosition(90);
                hangPower.update(0.2);
                engage = true;
            },
            () ->engage && engageTime.milliseconds() > 300
    );


}
