package dev.weaponboy.command_library.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

import dev.weaponboy.command_library.CommandLibrary.Commands.Command;
import dev.weaponboy.command_library.CommandLibrary.Commands.LambdaCommand;
import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;
import dev.weaponboy.command_library.CommandLibrary.Subsystem.SubSystem;
import dev.weaponboy.command_library.Hardware.AxonEncoder;
import dev.weaponboy.command_library.Hardware.ServoDegrees;
import dev.weaponboy.command_library.Hardware.collectionTarget;

public class Collection extends SubSystem{

    /**
     * slides constants
     * */
    double cmPerTick = 0.242;
    double targetPosition = 30*cmPerTick;
    double currentPosition = 0;
    double maxAccel = 120;
    double maxVelocity = 300;
    double accelDistance = maxVelocity/maxAccel;
    double powerFeedForwardConstant = (1/maxVelocity);
    int lastIndex = 0;
    ElapsedTime currentTime = new ElapsedTime();
    ArrayList<Double> motionProfile = new ArrayList<>();
    ArrayList<Double> Time = new ArrayList<>();
    double slideTime;
    double timeError =currentTime.milliseconds()-lastIndex;

    /**
     * linear rail constants
     * */
    double currentRailPosition;
    double railTargetPosition;
    double currentAxonWirePos;
    double lastAxonWirePos;
    final double spoolSize = 3.6; //in cm
    double railTimeToPosition;
    double rotationsForFullTravel = 20/(spoolSize*Math.PI);
    double timeForFullRotation = 540; // in ms
    double timePerCM = (rotationsForFullTravel*timeForFullRotation)/20;
    ElapsedTime railTime = new ElapsedTime();
    boolean runningToPosition = false;

    double waitForTransfer;
    double timePerDegree = (double) 720 /360;
    ElapsedTime initialTransfer = new ElapsedTime();

    /**
     * Motor and servos
     * */
    public DcMotorEx horizontalMotor;
    public ServoDegrees fourBarMainPivot = new ServoDegrees();
    public ServoDegrees fourBarSecondPivot= new ServoDegrees();
    public ServoDegrees griperRotate= new ServoDegrees();
    public ServoImplEx gripServo;
    public Servo linerRailServo;
    public ServoDegrees nest = new ServoDegrees();

    public AxonEncoder linearPosition = new AxonEncoder();

    public enum fourBar{
        preCollect,
        collect,
        dropNest,
        transferring,
        stowed,
    }
    public enum Nest{
        sample,
        specimen,
    }

    public enum gripper{
        drop,
        grab
    }

    public gripper gripperState = gripper.drop;
    public fourBar collectionState = fourBar.stowed;
    public Nest nestState = Nest.sample;

    double armLength = 9.6;
    double mainPivotHeight = 12.4;

    collectionTarget target = new collectionTarget();
    double lastClawY = 0;
    double currentClawY = 0;

    public final int maxSlideExtension = 640;

    public Collection(OpModeEX opModeEX) {
        registerSubsystem(opModeEX, nothing);
    }

    @Override
    public void execute() {
        executeEX();
        if(railTargetPosition != 0){
            updateRailPosition();
        }
    }

    @Override
    public void init() {

        horizontalMotor = getOpModeEX().hardwareMap.get(DcMotorEx.class, "horizontalMotor");
        horizontalMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        fourBarMainPivot.initServo("fourBarMainPivot", getOpModeEX().hardwareMap);
        fourBarSecondPivot.initServo("fourBarSecondPivot", getOpModeEX().hardwareMap);
        griperRotate.initServo("gripperRotate", getOpModeEX().hardwareMap);
        gripServo = getOpModeEX().hardwareMap.get(ServoImplEx.class, "gripServo");
        linerRailServo  = getOpModeEX().hardwareMap.get(ServoImplEx.class, "linearRailServo");
        nest.initServo("nest", getOpModeEX().hardwareMap);
        linearPosition.init(getOpModeEX().hardwareMap, "axon2");

        fourBarMainPivot.setRange(335);
        fourBarSecondPivot.setRange(new PwmControl.PwmRange(500, 2500), 270);
        griperRotate.setRange(new PwmControl.PwmRange(500, 2500), 270);
        gripServo.setPwmRange(new PwmControl.PwmRange(500, 2500));
        nest.setRange(new PwmControl.PwmRange(500, 2500), 270);

        fourBarMainPivot.setDirection(Servo.Direction.REVERSE);
        fourBarMainPivot.setOffset(36);

        nest.setDirection(Servo.Direction.REVERSE);
        nest.setOffset(5);

        griperRotate.setOffset(-10);
        griperRotate.setPosition(135);

        stow.execute();

    }

    public  Command grab = new LambdaCommand(
            () -> {},
            () -> {
                gripServo.setPosition(0.5);
                gripperState = gripper.grab;
            },
            () -> true
    );

    public Command drop = new LambdaCommand(
            () -> {},
            () -> {
                gripServo.setPosition(0);
                gripperState = gripper.grab;
            },
            () -> true
    );

    public LambdaCommand nestSample = new LambdaCommand(
            () -> {},
            () -> {
                nest.setPosition(135);
                nestState = Nest.sample;
            },
            () -> true
    );

    public LambdaCommand nestSpecimen = new LambdaCommand(
            () -> {},
            () -> {
                nest.setPosition(45);
                nestState = Nest.specimen;
            },
            () -> true
    );

    public Command kinModel(collectionTarget target){
        this.target = target;
        return kinModel;
    }

    public Command kinModel = new LambdaCommand(
            () -> {},
            () -> {
                double mainAngle = 0;
                double secondAngle = 0;
                double railTarget = 0;
                lastClawY = currentClawY;

                double secondPivotHeight = target.getZ() + armLength;

                if (secondPivotHeight > mainPivotHeight){
                    mainAngle = Math.toRadians(90) + (Math.toRadians(90) - (Math.acos((secondPivotHeight - mainPivotHeight) / armLength)));
                }else if (secondPivotHeight < mainPivotHeight){
                    mainAngle = Math.acos((secondPivotHeight - mainPivotHeight) / armLength);
                }

                currentClawY = armLength * Math.sin(mainAngle);
                secondAngle = 180 - Math.toDegrees(mainAngle);

                targetPosition += currentClawY - lastClawY;
                //spool ==40mm
                // cir 12.56cm

                fourBarMainPivot.setPosition(mainAngle);
                fourBarSecondPivot.setPosition(secondAngle);


            },
            () -> true
    );

    public  Command nothing = new LambdaCommand(
            () -> {},
            () -> {

            },
            () -> true
    );

    public  Command init = new LambdaCommand(
            () -> {
            },
            () -> {
                fourBarMainPivot.setPosition(160);
                fourBarSecondPivot.setPosition(128);
                griperRotate.setPosition(130);
            },
            () -> true
    );

   public Command collect = new LambdaCommand(
        () -> {},
        () -> {
            fourBarMainPivot.setPosition(96);
            fourBarSecondPivot.setPosition(6);
            collectionState = fourBar.collect;
        },
        () -> true
    );

    public Command stow = new LambdaCommand(
            () -> {
                initialTransfer.reset();
                waitForTransfer = Math.abs(fourBarMainPivot.getPositionDegrees() - 175)*timePerDegree;
            },
            () -> {
                fourBarMainPivot.setPosition(175);
                fourBarSecondPivot.setPosition(180);

                if (initialTransfer.milliseconds() > waitForTransfer){
                    griperRotate.setPosition(45);
                    collectionState = fourBar.stowed;
                }
            },
            () -> initialTransfer.milliseconds() > waitForTransfer
    );

    public Command transfer = new LambdaCommand(
            () -> {
                initialTransfer.reset();
                waitForTransfer = Math.abs(fourBarSecondPivot.getPositionDegrees() - 190)*timePerDegree;
            },
            () -> {
                fourBarMainPivot.setPosition(90);
                fourBarSecondPivot.setPosition(190);
                griperRotate.setPosition(135);

                collectionState = fourBar.transferring;

                if (initialTransfer.milliseconds() > waitForTransfer){
                    queueCommand(stow);
                }
            },
            () -> initialTransfer.milliseconds() > waitForTransfer
    );

    public Command dropNest = new LambdaCommand(
            () -> {

            },
            () -> {

                fourBarMainPivot.setPosition(191);
                fourBarSecondPivot.setPosition(189);
                griperRotate.setPosition(45);
                collectionState =fourBar.dropNest;
                nest.setOffset(5);
                nest.setPosition(135);

            },
            () -> true
    );

   public Command camera = new LambdaCommand(
            () -> {

            },
            () -> {
                fourBarMainPivot.setPosition(120);
                fourBarSecondPivot.setPosition(40);
                griperRotate.setPosition(135);
            },
            () -> true
    );

    public Command preCollect = new LambdaCommand(
            () -> {

            },
            () -> {
                fourBarMainPivot.setPosition(109);
                fourBarSecondPivot.setPosition(14);
                griperRotate.setPosition(135);
                collectionState =fourBar.preCollect;
            },
            () -> true
    );

//    public Command PID = new LambdaCommand(
//            () -> {
//
//            },
//            () -> {
//                horizontalMotor.setPower(slidePIDPower/2);            },
//            () -> true
//    );

   public LambdaCommand followMotionProfile = new LambdaCommand(
            () -> {
                currentTime.reset();
                lastIndex = 0;
            },
            () -> {
                while (Time.get(lastIndex) < currentTime.milliseconds()) {
                    lastIndex+=timeError;

                }

                double targetVelocity=motionProfile.get(lastIndex);
                double targetMotorPower=targetVelocity/powerFeedForwardConstant;
                horizontalMotor.setPower(targetMotorPower);
            },
            () ->slideTime> currentTime.milliseconds()
    );

    public void generateMotionProfile(double slideTarget) {
        this.targetPosition=slideTarget;
        slideTime = 0;
        double halfwayDistance = targetPosition / 2;
        double newAccelDistance = accelDistance;
        int decelCounter = 0;

        if (accelDistance > halfwayDistance) {
            newAccelDistance = halfwayDistance;
        }

        double newMaxVelocity = accelDistance * maxAccel;

        for (int i = 0; i < Math.abs(targetPosition - currentPosition); i++) {
            double targetVelocity;

            if (newAccelDistance > i) {

                int range = (int) Math.abs(newAccelDistance - i);

                double AccelSlope = (double) range / (double) Math.abs(newAccelDistance) * 100;

                AccelSlope = 100 - (AccelSlope * 0.01);

                targetVelocity = newMaxVelocity * AccelSlope;

                slideTime += (1 / targetVelocity) * 1000;

            }
            if (i + newAccelDistance > Math.abs(targetPosition - currentPosition)) {

                decelCounter++;

                int range = (int) Math.abs(newAccelDistance - decelCounter);

                double DeccelSlope = (double) range / Math.abs(newAccelDistance) * 100;

                DeccelSlope = DeccelSlope * 0.01;

                targetVelocity = newMaxVelocity * DeccelSlope;

                slideTime += (1 / targetVelocity) * 1000;

            } else {
                targetVelocity = newMaxVelocity;

                slideTime += (1 / targetVelocity) * 1000;
            }

            motionProfile.add(targetVelocity);
            Time.add(slideTime);

        }

    }

    public double getRailPosition() {
        return currentRailPosition;
    }

    public void setRailTargetPosition(double targetPosition) {
        this.railTargetPosition = targetPosition;
        updateRailPosition();
    }

    private void updateRailPosition(){

        double lastPosition = currentRailPosition;
        lastAxonWirePos = currentAxonWirePos;

        currentAxonWirePos = linearPosition.getPosition();

        double deltaPosition = lastAxonWirePos - currentAxonWirePos;
        double realDelta;
        double deltaCM;

        double spoolSize = 10.676;
        double cmPerDegree = spoolSize / 360;

        if ((lastAxonWirePos > 280 && currentAxonWirePos < 80) || (currentAxonWirePos > 280 && lastAxonWirePos < 80)){

            if (deltaPosition > 0){

                realDelta = findRealDelta(lastAxonWirePos, currentAxonWirePos);
                deltaCM = realDelta * cmPerDegree;
                currentRailPosition += deltaCM;

            } else if (deltaPosition < 0) {

                realDelta = findRealDelta(lastAxonWirePos, currentAxonWirePos);
                deltaCM = realDelta * cmPerDegree;
                currentRailPosition -= deltaCM;

            }

        } else {
            currentRailPosition -= deltaPosition*cmPerDegree;
        }

        runToPosition();

    }

    private void runToPosition(){

        double delta = railTargetPosition - currentRailPosition;

        if (Math.abs(delta) < 0.2){
            linerRailServo.setPosition(0.5);
            runningToPosition = false;
        }else if(Math.abs(delta) > 1 && !runningToPosition){
            railTimeToPosition = Math.abs(delta)*timePerCM;
            railTime.reset();
            if (delta > 0){ linerRailServo.setPosition(1);}else {linerRailServo.setPosition(0);}
            runningToPosition = true;
        }else if (railTime.milliseconds() >= railTimeToPosition && runningToPosition){
            linerRailServo.setPosition(0.5);
            runningToPosition = false;
        }

        updateRailPosition();
    }

    private static double findRealDelta(double last, double current){
        double realDelta;

        if (last > current){
            realDelta = current + (360 - last);
        }else {
            realDelta = last + (360 - current);
        }

        return realDelta;
    }

    public void disableServos(){
        fourBarMainPivot.disableServo();
        fourBarSecondPivot.disableServo();
        griperRotate.disableServo();
        gripServo.getController().pwmDisable();
        linerRailServo.getController().pwmDisable();
        nest.disableServo();
    }
}
