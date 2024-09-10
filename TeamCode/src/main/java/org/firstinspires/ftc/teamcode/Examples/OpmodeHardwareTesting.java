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

    int counter = 0;
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

        motor1.update(0.1);
        motor2.update(0.1);
        motor3.update(0.1);
        motor4.update(0.1);
    }

    @Override
    public void loop() {
        startTime = time.milliseconds();

        if(counter == 20){
            firstSetTime = time.milliseconds();
//            motor1.update(0.1);
//            motor2.update(0.1);
//            motor3.update(0.1);
//            motor4.update(0.1);

            motor1.update(0.3);
            motor2.update(0.3);
            motor3.update(0.3);
            motor4.update(0.3);

            counter++;
        }

        if (motor1.getSetPowerFuture().isDone() && counter > 20){
            System.out.println("first motor Done: " + motor1.getSetPowerFuture().isDone());
        }

        if (motor2.getSetPowerFuture().isDone() && counter > 20){
            System.out.println("second motor Done: " + motor2.getSetPowerFuture().isDone());
        }

        if (motor3.getSetPowerFuture().isDone() && counter > 20){
            System.out.println("third motor Done: " + motor3.getSetPowerFuture().isDone());
        }

        if (motor4.getSetPowerFuture().isDone() && counter > 20){
            System.out.println("forth motor Done: " + motor4.getSetPowerFuture().isDone());
        }

        counter++;

        System.out.println("loop time: " + ((time.milliseconds()-startTime)));

    }

}
