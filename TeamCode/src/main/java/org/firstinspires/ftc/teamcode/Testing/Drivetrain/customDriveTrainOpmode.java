package org.firstinspires.ftc.teamcode.Testing.Drivetrain;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;
import dev.weaponboy.command_library.Hardware.MotorEx;
import dev.weaponboy.command_library.Subsystems.DriveBase;

@TeleOp(name = "Drive_Only", group = "Testing")
public class customDriveTrainOpmode extends OpMode {

    MotorEx LF = new MotorEx();
    MotorEx RF = new MotorEx();
    MotorEx RB = new MotorEx();
    MotorEx LB = new MotorEx();

    @Override
    public void init() {
        LF.initMotor("LF", hardwareMap);
        RF.initMotor("RF", hardwareMap);
        LB.initMotor("LB", hardwareMap);
        RB.initMotor("RB", hardwareMap);

        LF.setDirection(DcMotorSimple.Direction.REVERSE);
        LB.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {

        drive(gamepad1.right_stick_y*0.5, (gamepad1.left_trigger - gamepad1.right_trigger)*0.4, -gamepad1.right_stick_x*0.5);

        if (gamepad1.a){
            LF.update(0.5);
        }else{
            LF.update(0);
        }

        if (gamepad1.y){
            LB.update(0.5);
        }else{
            LB.update(0);
        }

        if (gamepad1.b){
            RF.update(0.5);
        }else{
            RF.update(0);
        }

        if (gamepad1.x){
            RB.update(0.5);
        }else{
            RB.update(0);
        }

    }

    public void drive(double vertikal, double turn, double strafe){

        double denominator = Math.max(1, Math.abs(vertikal)+Math.abs(strafe)+Math.abs(turn));

        LF.update((vertikal-strafe-turn)/denominator);
        RF.update((vertikal+strafe+turn)/denominator);
        LB.update((vertikal+strafe-turn)/denominator);
        RB.update((vertikal-strafe+turn)/denominator);
    }
}
