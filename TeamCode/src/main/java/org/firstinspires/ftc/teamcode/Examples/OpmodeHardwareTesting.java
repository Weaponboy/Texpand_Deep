package org.firstinspires.ftc.teamcode.Examples;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class OpmodeHardwareTesting extends OpMode {

    Motor_Scheduler motor1 = new Motor_Scheduler();
    Motor_Scheduler motor2 = new Motor_Scheduler();
    Motor_Scheduler motor3 = new Motor_Scheduler();
    Motor_Scheduler motor4 = new Motor_Scheduler();

    DcMotor motor;

    ElapsedTime time = new ElapsedTime();

    boolean counter = false;
    double startTime =  0;
    double firstSetTime =  0;
    double endTime =  0;


    boolean firstDone = false;

    @Override
    public void init() {
        motor1.initMotor("motor", this.hardwareMap);
        motor2.initMotor("motor2", hardwareMap);
        motor3.initMotor("motor3", hardwareMap);
        motor4.initMotor("motor4", hardwareMap);

//        motor = hardwareMap.get(DcMotor.class, "motor");

        time.reset();
    }

    @Override
    public void loop() {
        startTime = time.milliseconds();

        if(!counter){
            firstSetTime = time.milliseconds();
//            motor1.update(0.1);
//            motor2.update(0.1);
//            motor3.update(0.1);
//            motor4.update(0.1);

            motor1.setPower(0.1);
            motor2.setPower(0.1);
            motor3.setPower(0.1);
            motor4.setPower(0.1);

            counter = true;
        }

//        if (motor1.getSetPowerFuture().isDone()){
////            motor1.update(0.2);
//            System.out.println("first motor Done: " + motor1.getSetPowerFuture().isDone());
//        }

        System.out.println("loop time: " + ((time.milliseconds()-startTime)));

    }

}
