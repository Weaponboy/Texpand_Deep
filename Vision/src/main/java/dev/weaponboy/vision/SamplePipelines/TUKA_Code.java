package dev.weaponboy.vision.SamplePipelines;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class TUKA_Code extends OpMode {

    Gamepad lastGamepad = new Gamepad();
    Gamepad currentGamepad = new Gamepad();

    DcMotor left_Drive;
    DcMotor right_Drive;

    DcMotor arm_Motor;
    DcMotor wrist_Motor;

    Servo intakeServo;
    Servo clippingGripper;

    double vertical;
    double turning;

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

    private static final double CLAW_OPEN_POSITION = 0.8;
    private static final double CLAW_CLOSED_POSITION = 0;

    enum ArmState{
        lowBasket,
        collectSample,
        preClip,
        clip,
        wallCollect,
        wallUnhook,
    }

    ArmState armState = ArmState.collectSample;

    @Override
    public void init() {
        left_Drive = hardwareMap.get(DcMotor.class, "left_Drive");
        right_Drive = hardwareMap.get(DcMotor.class,"right_Drive");

        arm_Motor = hardwareMap.get(DcMotor.class,"arm_Motor");
        wrist_Motor = hardwareMap.get(DcMotor.class,"wrist_Motor");

        intakeServo = hardwareMap.get(Servo.class,"intakeServo");
        clippingGripper = hardwareMap.get(Servo.class,"clippingGripper");

        left_Drive.setDirection(DcMotorSimple.Direction.REVERSE);

        left_Drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_Drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        arm_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        wrist_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wrist_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    @Override
    public void loop() {

        lastGamepad.copy(currentGamepad);
        currentGamepad.copy(gamepad1);

        vertical = gamepad1.right_stick_y;
        turning = gamepad1.left_stick_x;

        left_Drive.setPower(vertical + turning);
        right_Drive.setPower(vertical - turning);

        if (gamepad1.dpad_left) {
            intakeServo.setPosition(1);
        } else if (gamepad1.dpad_right) {
            intakeServo.setPosition(0);
        } else {
            intakeServo.setPosition(0.5);
        }

        if (gamepad1.left_bumper) {
            clippingGripper.setPosition(CLAW_OPEN_POSITION);
        }else if (gamepad1.right_bumper){
            clippingGripper.setPosition(CLAW_CLOSED_POSITION);
        }

        if (currentGamepad.b && !lastGamepad.b && armState == ArmState.collectSample) {
            armState = ArmState.lowBasket;
            updateArm();
        } else if (currentGamepad.b && !lastGamepad.b && armState != ArmState.collectSample){
            armState = ArmState.collectSample;
            updateArm();
        }

        if (currentGamepad.a && !lastGamepad.a && armState != ArmState.wallCollect) {
            armState = ArmState.wallCollect;
            updateArm();
        } else if (currentGamepad.a && !lastGamepad.a && armState == ArmState.wallCollect) {
            armState = ArmState.wallUnhook;
            updateArm();
        }

        if (currentGamepad.y && !lastGamepad.y &&  armState != ArmState.wallUnhook) {
            armState = ArmState.preClip;
            updateArm();
        } else if (currentGamepad.y && !lastGamepad.y &&  armState == ArmState.preClip) {
            armState = ArmState.clip;
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
                wrist_Motor.setTargetPosition(WRIST_POSITION_SPEC);
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

            default:
        }
    }
}
