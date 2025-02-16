package org.firstinspires.ftc.teamcode.Testing.Teleops;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;

import dev.weaponboy.command_library.Hardware.MotorEx;
import dev.weaponboy.command_library.Hardware.ServoDegrees;

@TeleOp(name = "Hang_Test", group = "Testing")
public class hangTest extends OpMode {
    Servo hang1;
    Servo hang2;
    public IMU imu;



    public ServoDegrees PTO = new ServoDegrees();

    MotorEx hangPower = new MotorEx();
    MotorEx hangPower2 = new MotorEx();
    MotorEx hangPower3 = new MotorEx();

    @Override
    public void init() {
        imu = hardwareMap.get(IMU.class, "imu");

        hang1=hardwareMap.get(Servo.class,"hang1");
        hang2=hardwareMap.get(Servo.class,"hang2");

        hangPower2.initMotor("slideMotor", hardwareMap);
        hangPower2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangPower2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hangPower3.initMotor("slideMotor2", hardwareMap);
        hangPower3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangPower3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        PTO.initServo("hangPTO", hardwareMap);
        PTO.setRange(new PwmControl.PwmRange(500, 2500), 270);
//        gripServo.setRange(180);
//        griperRotate.setRange(new PwmControl.PwmRange(500, 2500), 180);

        RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection = RevHubOrientationOnRobot.LogoFacingDirection.DOWN;
        RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoFacingDirection, usbFacingDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));

        imu.resetYaw();
//
        hangPower.initMotor("hangPower", hardwareMap);
        hangPower.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        PTO.setDirection(Servo.Direction.REVERSE);

        hang1.setPosition(0.5);
        hang2.setPosition(0.5);

        hangPower2.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    @Override
    public void loop() {

        if (gamepad1.y){
            hang1.setPosition(0);
            hang2.setPosition(0);

        }else if (gamepad1.b){
            hang1.setPosition(1);
            hang2.setPosition(1);

        }else {
            hang1.setPosition(0.5);
            hang2.setPosition(0.5);
        }

        if (gamepad1.start){
            PTO.setPosition(118);
        }

        double power = 0;

        if (gamepad1.a){
            PTO.setPosition(90);
            hangPower.update(0);
        }else if (gamepad1.right_bumper){
            power = -1;
        } else if (gamepad1.left_bumper && (Math.abs(hangPower2.getCurrentPosition())) + (Math.abs(hangPower3.getCurrentPosition()))/2 < 670) {
            power = 1;
        }else {
            power = 0;
        }

        hangPower.update(power);

        imu.resetYaw();

        telemetry.addData("rpm", hangPower.getVelocity());
        telemetry.addData("hight",(Math.abs(hangPower2.getCurrentPosition())) + (Math.abs(hangPower3.getCurrentPosition())));
        telemetry.addData("roll",imu.getRobotYawPitchRollAngles().getRoll());
        telemetry.addData("pitch",imu.getRobotYawPitchRollAngles().getPitch());
        telemetry.update();

    }
}
