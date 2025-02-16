//package Users.administrator.Desktop.TeamCode.src.main.java.org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//
//@TeleOp
//
//public class First_Class extends OpMode {
//
//    DcMotor left_drive;
//    DcMotor right_drive = null;
//    DcMotor arm_motor = null;
//    Servo roller = null;
//    double left;
//    double right;
//    int delivery = 200;
//    int collect = 0;
//
//
//    @Override
//    public void init() {
//        //
//        left_drive = hardwareMap.get(DcMotor.class, "left_drive");
//        right_drive = hardwareMap.get(DcMotor.class, "right_drive");
//        arm_motor = hardwareMap.get(DcMotor.class, "arm_motor");
//        roller = hardwareMap.get(Servo.class, "roller");
//
//        arm_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        arm_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        right_drive.setDirection(DcMotorSimple.Direction.REVERSE);
//        left_drive.setDirection(DcMotorSimple.Direction.FORWARD);
//        // rightDrive.setDirection(DcMotor.Direction.FORWARD) ;
//
//        left_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        right_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//    }
//
//    @Override
//    public void loop() {
//
//        double turn = 0;
//        double forward = 0;
//        turn = gamepad1.right_stick_x;
//        forward = -gamepad1.left_stick_y;
//
//        left_drive.setPower(forward+turn);
//        right_drive.setPower(forward-turn);
//
//        //move to collect
//        if (gamepad1.dpad_up) {
//            arm_motor.setTargetPosition(collect);
//
//            arm_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            arm_motor.setPower(0.5);
//
//        }else if (gamepad1.dpad_down) {
//            arm_motor.setTargetPosition(delivery);
//
//            arm_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            arm_motor.setPower(0.5);
//        } else {
//            arm_motor.setPower(0);
//        }
//
//        //open gripper
//        if (gamepad1.left_bumper){
//            roller.setPosition(0);
//        }
//
//        //close gripper
//        if (gamepad1.right_bumper) {
//            roller.setPosition(1);
//        }
//
//        telemetry.addData("arm pos",arm_motor.getCurrentPosition());
//        telemetry.update();
//
//    }
//
//}