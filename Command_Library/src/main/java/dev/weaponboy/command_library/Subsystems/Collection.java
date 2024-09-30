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
import dev.weaponboy.nexus_pathing.PathingUtility.PIDController;

public class Collection extends SubSystem{

    double cmPerTick = 0.242;
    double targetPosition = 30*cmPerTick;
    double currentPosition = 0;
    double maxAccel = 120;
    double maxVelocity = 300;
    double accelDistance = maxVelocity/maxAccel;
    double powerFeedForwardConstant = (1/maxVelocity);
    int lastIndex = 0;

    double currentRailPosition;
    double currentAxonWirePos;

    public double getLastAxonWirePos() {
        return lastAxonWirePos;
    }

    public double getCurrentAxonWirePos() {
        return currentAxonWirePos;
    }

    double lastAxonWirePos;



    ElapsedTime currentTime = new ElapsedTime();
    ArrayList<Double> motionProfile = new ArrayList<>();
    ArrayList<Double> Time = new ArrayList<>();
    double slideTime;
    double timeError =currentTime.milliseconds()-lastIndex;


    public DcMotorEx horizontalMotor;
    public ServoDegrees fourBarMainPivot = new ServoDegrees();
    public ServoDegrees fourBarSecondPivot= new ServoDegrees();
    public ServoDegrees griperRotate= new ServoDegrees();
    public ServoImplEx gripServo;
    public Servo linerRailServo;
    public ServoDegrees nest = new ServoDegrees();

    public AxonEncoder linearPosition = new AxonEncoder();

    PIDController slidePID =new PIDController(0.015,0,0.0005);

    public enum fourBar{
        preCollect,
        collect,
        transfer,
        stowed,
    }
    public enum Nest{
        sample,
        specimine,
    }

    public fourBar collectionState = fourBar.stowed;
    public Nest nestState = Nest.sample;

    double armLength = 9.6;
    double mainPivotHeight = 12.4;

    collectionTarget target = new collectionTarget();
    double lastClawY = 0;
    double currentClawY = 0;

    public final int maxSlideExtension = 640;

    public Collection(OpModeEX opModeEX) {
        registerSubsystem(opModeEX,nothing);
    }

    @Override
    public void execute() {
        executeEX();
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
//     void getMotorPozition(){
//        double PIDTargetPosition =160;
//        double slideError=PIDTargetPosition-horizontalMotor.getCurrentPosition();
//        double P=slideError*0.015;
//        double D=slideError*0.0005;
//        double slidePIDPower=P+D;

    //    horazontolPozition=horizontalMotor.getCurrentPosition();
  //  }



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
            () -> {
            },
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

    public  Command grip = new LambdaCommand(
            () -> {
            },
            () -> {
                gripServo.setPosition(0.5);
            },
            () -> true
    );

  public Command drop = new LambdaCommand(
            () -> {

            },
            () -> {
                gripServo.setPosition(0);
            },
            () -> true
    );
    public LambdaCommand nestSample = new LambdaCommand(
            () -> System.out.println("init"),
            () -> {
                nest.setPosition(135);
                nestState =Nest.sample;
            },
            () -> true
    );
    public LambdaCommand nestSpecimine = new LambdaCommand(
            () -> System.out.println("init"),
            () -> {
                nest.setPosition(45);
                nestState =Nest.specimine;

            },
            () -> true
    );

   public Command Collect = new LambdaCommand(
       () -> {

            },
        () -> {
            fourBarMainPivot.setPosition(96);
            fourBarSecondPivot.setPosition(6);
            collectionState =fourBar.collect;
        },
        () -> true
    );

    public   Command stow = new LambdaCommand(
            () -> {

            },
            () -> {
                fourBarMainPivot.setPosition(175);
                fourBarSecondPivot.setPosition(180);
                griperRotate.setPosition(45);
                collectionState = fourBar.stowed;
            },
            () -> true
    );

    public   Command transfer = new LambdaCommand(
            () -> {

            },
            () -> {

                fourBarMainPivot.setPosition(191);
                fourBarSecondPivot.setPosition(189);
                griperRotate.setPosition(45);
                collectionState =fourBar.transfer;
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
//        this.currentRailPosition = targetPosition;
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
