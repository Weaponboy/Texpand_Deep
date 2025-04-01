package dev.weaponboy.command_library.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.ArrayList;

import dev.weaponboy.command_library.CommandLibrary.Commands.Command;
import dev.weaponboy.command_library.CommandLibrary.Commands.LambdaCommand;
import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;
import dev.weaponboy.command_library.CommandLibrary.Subsystem.SubSystem;
import dev.weaponboy.command_library.Hardware.DistanceSensor;
import dev.weaponboy.command_library.Hardware.SensorReadings;

public class Odometry extends SubSystem {

    DistanceSensor backLeft = new DistanceSensor();
    DistanceSensor backRight = new DistanceSensor();
    DistanceSensor right = new DistanceSensor();

    ArrayList<SensorReadings> sensorReadings = new ArrayList<>();

    DcMotorEx leftPod;
    DcMotorEx rightPod;
    DcMotorEx backPod;

    double X, Y, Heading;
    int startHeading;
    public double otherHeading;

    double lastRightPod, lastLeftPod, lastBackPod;
    public double currentRightPod, currentLeftPod, currentBackPod;
    public double rightPodPos, leftPodPos, backPodPos;

    double podTicks = 2000;
    double wheelRadius = 2.4;
    double trackWidth = 24.2;
    double backPodOffset = 9.8;

    double ticksPerCM = ((2.0 * Math.PI) * wheelRadius)/podTicks;
    double cmPerDegreeX = (double) (2) / 360;
    double cmPerDegreeY = ((2.0 * Math.PI) * backPodOffset) / 360;

    double currentXVelocity = 0;
    double currentYVelocity = 0;

    boolean sampleReset = true;

    public boolean isRunningDistanceSensorReset() {
        return runningDistanceSensorReset;
    }

    public void runDistanceSensorReset(boolean sampleReset) {
        runningDistanceSensorReset = true;
        this.sampleReset = sampleReset;
        resetCounter = 0;
        sensorReadings.clear();
    }

    boolean runningDistanceSensorReset = false;
    int resetCounter = 0;

    public Odometry(OpModeEX opModeEX) {
        registerSubsystem(opModeEX, updateLineBased);
    }

    public void startPosition(double X, double Y, int Heading){
        this.X = X;
        this.Y = Y;
        this.startHeading = Heading;
        this.Heading = Math.toRadians(Heading);
    }

    @Override
    public void init() {
        leftPod = getOpModeEX().hardwareMap.get(DcMotorEx.class, "RF");
        rightPod = getOpModeEX().hardwareMap.get(DcMotorEx.class, "RB");
        backPod = getOpModeEX().hardwareMap.get(DcMotorEx.class, "LB");

        backRight.init(getOpModeEX().hardwareMap, "backRight");
        backLeft.init(getOpModeEX().hardwareMap, "backLeft");

        backRight.setOffset(-180);
        backLeft.setOffset(-180);

        right.init(getOpModeEX().hardwareMap, "right");

        right.setOffset(-200);
    }

    public double headingError(double targetHeading){
        return Heading-targetHeading;
    }

    @Override
    public void execute() {

        executeEX();
        updateVelocity();

        if (runningDistanceSensorReset){

            resetCounter++;
            sensorReadings.add(new SensorReadings(backRight.getPosition(), backLeft.getPosition(), right.getPosition()));

//            System.out.println(sensorReadings.size());
//            System.out.println(resetCounter);

            if (resetCounter > 20){

                runningDistanceSensorReset = false;
                SensorReadings averaged;

                double backRight = 0;
                double backLeft = 0;
                double right = 0;

                for (SensorReadings reading: sensorReadings){
                    backRight += reading.getSen1();
                    backLeft += reading.getSen2();
                    right += reading.getSen3();

//                    System.out.println(reading.getSen1());
//                    System.out.println(reading.getSen2());
//                    System.out.println(reading.getSen3());
                }

                averaged = new SensorReadings((backRight/resetCounter)*0.1, (backLeft/resetCounter)*0.1, (right/resetCounter)*0.1);

//                System.out.println("Avg sen 1: " + averaged.getSen1());
//                System.out.println("Avg sen 2: " + averaged.getSen2());
//                System.out.println("Avg sen 3: " + averaged.getSen3());

                final double distanceFromRobotCenterToSensor = 11;
                final double distanceBetweenSensors = 13.2;

                double readingDifference = averaged.getSen2() - averaged.getSen1();

                double headingError = Math.atan(readingDifference / distanceBetweenSensors);

                double newHeading;
                double newY;
                double newX;

                if (sampleReset){
                    newHeading = 180 + Math.toDegrees(headingError);

                    newY = 360 - (Math.cos(Math.abs(headingError))*(averaged.getSen3() + distanceFromRobotCenterToSensor));
                    newX = 360 - (((averaged.getSen1() + averaged.getSen2())/2) + 17.5);
                }else{
                    newHeading = 90 + Math.toDegrees(headingError);

                    newX = 360 - (Math.cos(Math.abs(headingError))*(averaged.getSen3() + distanceFromRobotCenterToSensor));
                    newY = (((averaged.getSen1() + averaged.getSen2())/2) + 17.5);
                }



                X = newX;
                Y = newY;
                Heading = Math.toRadians(newHeading);

            }
        }
    }

    public double X (){
        return X;
    }

    public double Y (){
        return Y;
    }

    public double Heading (){
        return Math.toDegrees(Heading);
    }

    public double getYVelocity(){
        return currentYVelocity;
    }

    public double getXVelocity(){
        return currentXVelocity;
    }

    public void updateVelocity(){
        double RRXError = ticksPerCM * ((-rightPod.getVelocity()+(-leftPod.getVelocity()))/2);
        double RRYError = ticksPerCM * -backPod.getVelocity();

//        currentXVelocity = RRXError;
//        currentYVelocity = RRYError;

        currentXVelocity = RRXError * Math.cos(Heading) - RRYError * Math.sin(Heading);
        currentYVelocity = RRXError * Math.sin(Heading) + RRYError * Math.cos(Heading);

    }

    public LambdaCommand update = new LambdaCommand(
            () -> {},
            () -> {
                //need to code this
                //prob some constant accel loc code
            },
            () -> false
    );

    public LambdaCommand updateLineBased = new LambdaCommand(
            () -> {},
            () -> {

//                System.out.println("execute odometry update line based");

                lastBackPod = currentBackPod;
                lastLeftPod = currentLeftPod;
                lastRightPod = currentRightPod;

                currentBackPod = -backPod.getCurrentPosition();
                currentLeftPod = -leftPod.getCurrentPosition();
                currentRightPod = -rightPod.getCurrentPosition();

                double deltaRight = currentRightPod - lastRightPod;
                double deltaLeft = currentLeftPod - lastLeftPod;
                double deltaBack = currentBackPod - lastBackPod;

                double deltaHeading = (ticksPerCM * (deltaRight - deltaLeft)) / (trackWidth+0.22);
                Heading += deltaHeading;

                if (Math.toDegrees(Heading) < 0){
                    Heading = Math.toRadians(360 - Math.toDegrees(Heading));
                } else if (Math.toDegrees(Heading) > 360) {
                    Heading = Math.toRadians(Math.toDegrees(Heading) - 360);
                }

                double deltaX = ((((deltaRight+deltaLeft)*ticksPerCM)/2)) + (Math.toDegrees(deltaHeading) * cmPerDegreeX);
                double deltaY = (ticksPerCM * deltaBack) - (Math.toDegrees(deltaHeading) * cmPerDegreeY);

//                X += deltaX;
//                Y += deltaY;

                X += deltaX * Math.cos(Heading) - deltaY * Math.sin(Heading);
                Y += deltaX * Math.sin(Heading) + deltaY * Math.cos(Heading);

                //4165 back pod 180

//                updatePodReadings();
//                leftPod.update(0);
//                rightPod.update(0);
//                backPod.update(0);

//                System.out.println(Math.abs(rightPod.getTimeCompleted() - leftPod.getTimeCompleted())/1000000);

            },
            () -> true
    );

    public Command resetPosition(double X, double Y, int Heading){
        this.X = X;
        this.Y = Y;
        startHeading = Heading;
        return resetPosition;
    }

    private final LambdaCommand resetPosition = new LambdaCommand(
            () -> {},
            () -> {

            },
            () -> false
    );

    private void updatePodReadings(){
//        leftPod.updatePosition();
//        rightPod.updatePosition();
//        backPod.updatePosition();
    }


}