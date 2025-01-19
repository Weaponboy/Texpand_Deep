//package org.firstinspires.ftc.teamcode.Testing.Teleops;
//
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//public class AutoNovaTeleop extends OpMode {
//
//    ElapsedTime timer = new ElapsedTime();
//    boolean clippingSpec = false;
//
//    @Override
//    public void init() {
//
//    }
//
//    @Override
//    public void loop() {
//        if (gamepad1.a && !clippingSpec){
//            armPosition = -1500;
//            slidePosition = 300;
//            elbow.setPosition(ELBOW_FOLDED_OUT);
//            wrist.setPosition(WRIST_MIDDLE);
//
//            //reset timer and start the clipping process
//            timer.reset();
//            clippingSpec = true;
//        }
//
//        if (clippingSpec && timer.milliseconds() > 500 && timer.milliseconds() < 600){
//
//            //open gripper after the arm is in position
//            gripper.setPosition(GRIPPER_OPEN);
//
//        }else if (clippingSpec && timer.milliseconds() > 500 && timer.milliseconds() < 600){
//
//            //finish the clipping process with the final set position and set clipping to false
//            clippingSpec = false;
//
//            armPosition = -100;
//            slidePosition = 0;
//            elbow.setPosition(ELBOW_FOLDED_IN);
//            wrist.setPosition(WRIST_MIDDLE);
//
//        }
//    }
//
//}
