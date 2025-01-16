//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//
//@TeleOp(name = "Encoder")
//public class Encoder extends OpMode {
//
//    DcMotor motor2;
//    double ticks = 2300;
//    int targetPosition;
//
//    @Override
//    public void init() {
//        motor2 = hardwareMap.get(DcMotor.class, "arm_motor");
//
//        telemetry.addData("Hardware:","Initialized");
//
//        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//    }
//
//    @Override
//    public void loop() {
//
//        if(gamepad1.a) {
//            motor2.setPower(-0.5);
//            targetPosition = motor2.getCurrentPosition();
//        }else if(gamepad1.b) {
//            motor2.setPower(0.5);
//            targetPosition = motor2.getCurrentPosition();
//        } else {
//            encoder(targetPosition);
//        }
//
//        telemetry.addData("motor position", motor2.getCurrentPosition());
//        telemetry.update();
//
//    }
//
//    public void encoder(int turnage) {
//        motor2.setTargetPosition(turnage);
//        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        motor2.setPower(0.5);
//    }
//}