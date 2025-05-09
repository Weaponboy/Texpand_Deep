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
import dev.weaponboy.command_library.Hardware.AxonEncoder;
import dev.weaponboy.command_library.Hardware.MotorEx;
import dev.weaponboy.command_library.Hardware.ServoDegrees;
import dev.weaponboy.nexus_pathing.PathingUtility.PIDController;

public class Hang extends SubSystem {

    public IMU imu;

    public Servo hang1;
    public Servo hang2;

    boolean servoActive = false;
    boolean engage = false;
    boolean stopServoActive = false;

    public AxonEncoder hang1Right = new AxonEncoder();
    public AxonEncoder hang1Left = new AxonEncoder();

    public void setSlideposition(double slideposition) {
        this.slideposition = slideposition;
    }

    double slideposition;
    enum hangState{
        pulldown,
        hold,
        abort
    }

    ElapsedTime engageTime = new ElapsedTime();
    ElapsedTime servoactive = new ElapsedTime();

    public hangState hangstate = hangState.pulldown;

    double targetTilt = -18.4;

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
        hang1.setDirection(Servo.Direction.REVERSE);

        hang1Left.init(getOpModeEX().hardwareMap, "leftHang");
        hang1Right.init(getOpModeEX().hardwareMap, "rightHang");

        hang1Left.setOffset(207);
        //target 340

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



    public LambdaCommand hang = new LambdaCommand(
            () -> {
                hangPower.update(0);
            },
            () -> {

                if (hangstate == hangState.pulldown && engage && engageTime.milliseconds() > 300 && imu.getRobotYawPitchRollAngles().getPitch() < targetTilt+0.5){
                    hangPower.update(-1);
                    hangPower2.update(-0.4);
                    hangPower3.update(-0.4);
                    hangstate = hangState.hold;
                    servoActive = false;

                    hang2.setPosition(0.5);
                    hang1.setPosition(0.5);
//                    stopServoActive = true;
//                    servoactive.reset();
                }

//                if (stopServoActive && servoactive.milliseconds() > 400){
//                    servoActive = false;
//                    stopServoActive = false;
//                }

                if (hangstate == hangState.hold && slideposition < 0){
                    hangPower.update(-0.4 + adjustment.calculate(2, slideposition));
                }

            },
            () -> false
    );

    public LambdaCommand Engage  = new LambdaCommand(
            () -> {
                engageTime.reset();
                queueCommand(hang);
            },
            () -> {
                PTO.setPosition(90);
                hangPower.update(0.2);
                engage = true;
                servoActive = true;
            },
            () -> engage && engageTime.milliseconds() > 300
    );


}
