package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous
public class SpecAuto extends LinearOpMode {

    DcMotor left_Drive;
    DcMotor right_Drive;

    DcMotor arm_Motor;
    DcMotor wrist_Motor;
    Servo intakeservo;
    Servo clippingGripper;

    private static final int ARM_POSITION_INIT = 300;
    private static final int ARM_POSITION_INTAKE = 450;
    private static final int ARM_POSITION_WALL_GRAB = 1100;
    private static final int ARM_POSITION_WALL_UNHOOK = 1700;
    private static final int ARM_POSITION_HOVER_HIGH = 2600;
    private static final int ARM_POSITION_CLIP_HIGH = 2100;
    private static final int ARM_POSITION_LOW_BASKET = 2500;

    private static final int WRIST_POSITION_INIT = 0;
    private static final int WRIST_POSITION_SAMPLE = 270;
    private static final int WRIST_POSITION_SPEC = 10;

    private static final double CLAW_OPEN_POSITION = 0.55;
    private static final double CLAW_CLOSED_POSITION = 0.7;

    enum ArmState {
        init,
        lowBasket,
        collectSample,
        preClip,
        clip,
        wallConnect,
        wallUnhook,
        wallCollect,
    }

    ArmState armState = ArmState.collectSample;

    ElapsedTime armTimer = new ElapsedTime();
    double wheelCir = 30;//in Cm
    double tickPerRev = 537;
    double tickPerCM = tickPerRev / wheelCir;

    double turningCircle = 40 * Math.PI;
    double cmPerDegree = turningCircle / 360;


    @Override
    public void runOpMode() throws InterruptedException {

        left_Drive = hardwareMap.get(DcMotor.class, "left_Drive");
        right_Drive = hardwareMap.get(DcMotor.class, "right_Drive");

        left_Drive.setDirection(DcMotorSimple.Direction.REVERSE);

        left_Drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_Drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left_Drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_Drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        arm_Motor = hardwareMap.get(DcMotor.class, "arm_Motor");
        wrist_Motor = hardwareMap.get(DcMotor.class, "wrist_Motor");

        intakeservo = hardwareMap.get(Servo.class, "intake_Servo");
        clippingGripper = hardwareMap.get(Servo.class, "clip_Servo");

        left_Drive.setDirection(DcMotorSimple.Direction.REVERSE);

        arm_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        arm_Motor.setTargetPosition(ARM_POSITION_INIT);

        wrist_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wrist_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        wrist_Motor.setTargetPosition(WRIST_POSITION_INIT);

        clippingGripper.setPosition(CLAW_CLOSED_POSITION);

        waitForStart();

        setArmState(ArmState.preClip, 1500);

        driveDistance(100);

        setArmState(ArmState.clip, 1500);

        clippingGripper.setPosition(CLAW_OPEN_POSITION);
        armTimer.reset();

        while (armTimer.milliseconds() < 500) {}

        driveDistance(-40);

        setArmState(ArmState.init, 1000);
    }

    public void setArmState(ArmState state, double waitTime) {
        armState = state;
        armTimer.reset();

        while (armTimer.milliseconds() > 1000) {
            updateArm();
        }
    }

    public void updateArm(){
        switch (armState){
            case clip:
                wrist_Motor.setTargetPosition(WRIST_POSITION_SPEC);
                arm_Motor.setTargetPosition(ARM_POSITION_CLIP_HIGH);
                wrist_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wrist_Motor.setPower(0.5);
                arm_Motor.setPower(0.5);
                break;
            case preClip:
                wrist_Motor.setTargetPosition(WRIST_POSITION_SAMPLE);
                arm_Motor.setTargetPosition(ARM_POSITION_HOVER_HIGH);
                wrist_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wrist_Motor.setPower(0.5);
                arm_Motor.setPower(0.5);
                break;
            case wallCollect:
                wrist_Motor.setTargetPosition(WRIST_POSITION_SAMPLE);
                arm_Motor.setTargetPosition(ARM_POSITION_WALL_GRAB);
                wrist_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wrist_Motor.setPower(0.5);
                arm_Motor.setPower(0.5);
                break;
            case wallUnhook:
                wrist_Motor.setTargetPosition(WRIST_POSITION_SAMPLE);
                arm_Motor.setTargetPosition(ARM_POSITION_WALL_UNHOOK);
                wrist_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wrist_Motor.setPower(0.5);
                arm_Motor.setPower(0.5);
                break;
            case collectSample:
                wrist_Motor.setTargetPosition(WRIST_POSITION_SAMPLE);
                arm_Motor.setTargetPosition(ARM_POSITION_INTAKE);
                wrist_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wrist_Motor.setPower(0.5);
                arm_Motor.setPower(0.5);
                break;
            case lowBasket:
                wrist_Motor.setTargetPosition(WRIST_POSITION_SAMPLE);
                arm_Motor.setTargetPosition(ARM_POSITION_LOW_BASKET);
                wrist_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wrist_Motor.setPower(0.5);
                arm_Motor.setPower(0.5);
                break;
            case init:
                wrist_Motor.setTargetPosition(WRIST_POSITION_INIT);
                arm_Motor.setTargetPosition(ARM_POSITION_INIT);
                wrist_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wrist_Motor.setPower(0.5);
                arm_Motor.setPower(0.5);
                break;

            default:
        }
    }

    public void turnDegrees ( double degrees){

        double targetCM = degrees * cmPerDegree;
        int motorTarget = (int) (targetCM * tickPerCM);

        left_Drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_Drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left_Drive.setTargetPosition(motorTarget);
        right_Drive.setTargetPosition(-motorTarget);

        left_Drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_Drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (left_Drive.isBusy()) {

        }


    }

    public void driveDistance ( double targetCM){

        int motorTarget = (int) (targetCM * tickPerRev);

        left_Drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_Drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left_Drive.setTargetPosition(motorTarget);
        right_Drive.setTargetPosition(motorTarget);

        left_Drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_Drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (left_Drive.isBusy()) {

        }

    }

}