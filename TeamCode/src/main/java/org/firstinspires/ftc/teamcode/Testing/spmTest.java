package org.firstinspires.ftc.teamcode.Testing;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Examples.OpMode;

@TeleOp
public class spmTest extends LinearOpMode {
    Servo axon;
    @Override
    public void runOpMode() throws InterruptedException {
axon=hardwareMap.get(Servo.class, "axon");
waitForStart();
        while (opModeIsActive() ){
            axon.setPosition(0.5);
        }

    }
}