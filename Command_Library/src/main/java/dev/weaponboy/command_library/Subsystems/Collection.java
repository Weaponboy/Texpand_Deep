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
    double targetPosition = 50;
    double currentPosition = 0;

    double maxAccel = 1400;
    double maxVelocity = 140;
    double acceldistance = (maxVelocity * maxVelocity) / (maxAccel*2);
    public ServoDegrees griperSev =new ServoDegrees();
    public ServoDegrees mainPivot=new ServoDegrees();
    public ServoDegrees secondPivot = new ServoDegrees();
    public ServoDegrees linierRail= new ServoDegrees();

    ArrayList<Double> motionProfile = new ArrayList<>();
    ArrayList<Double> positions = new ArrayList<>();
    ArrayList<Double> time = new ArrayList<>();
    ElapsedTime currentTime = new ElapsedTime();
    ElapsedTime flipBackTimer = new ElapsedTime();

    int lastIndex = 0;
    double slidetime;
    double veloToMotorPower = 1/maxVelocity;

    public final int maxSlideHeight = 1720;
    public final int maxSlideHeightCM = 53;
    private final int ticksPerCm = maxSlideHeight / maxSlideHeightCM;
    public final double CMPerTick = (double) maxSlideHeightCM / maxSlideHeight;

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
        stowed
    }

    public enum Nest{
        sample,
        specimen
    }

    public enum slideState{
        moving,
        manuel
    }

    public enum gripper{
        drop,
        grab
    }

    private slideState slidesState = slideState.manuel;
    private gripper gripperState = gripper.drop;
    private fourBar collectionState = fourBar.stowed;
    private Nest nestState = Nest.sample;

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

        if (nestState == Nest.sample){
            nest.setPosition(135);
        } else if (nestState == Nest.specimen) {
            nest.setPosition(45);
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
                gripperState = gripper.drop;
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
            () -> {},
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
            () -> {},
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
                fourBarSecondPivot.setPosition(25);
                griperRotate.setPosition(135);
                collectionState =fourBar.preCollect;
            },
            () -> true
    );

    public LambdaCommand followMotionPro = new LambdaCommand(
            () -> {
                currentTime.reset();
                lastIndex = 0;
                slidesState = slideState.moving;
            },
            ()-> {

                if (lastIndex >= time.size()){
                    if (targetPosition > horizontalMotor.getCurrentPosition()*CMPerTick){
                        while (positions.get(lastIndex-time.size()) < horizontalMotor.getCurrentPosition()*CMPerTick){
                            lastIndex++;
                        }
                    } else if (targetPosition < horizontalMotor.getCurrentPosition()*CMPerTick) {
                        while (positions.get(lastIndex-time.size()) > horizontalMotor.getCurrentPosition()*CMPerTick){
                            lastIndex++;
                        }
                    }
                }else {

                    if (time.get(lastIndex) < currentTime.milliseconds()) {
                        lastIndex++;
                    }

                }

                double targetVelocity = motionProfile.get(lastIndex);
                double targetMotorPower = targetVelocity*veloToMotorPower;

                horizontalMotor.setPower(targetMotorPower);

                System.out.println("slideMotor" + targetMotorPower);
                System.out.println("slideMotor" + lastIndex );
                System.out.println("slideMotor" + motionProfile.size());

                if (lastIndex >= motionProfile.size()-1){
                    slidesState = slideState.manuel;
                }

            },
            ()-> lastIndex >= motionProfile.size()-1
    );

    public void genMotionProfile (double slideTarget){

        time.clear();
        motionProfile.clear();
        slidetime = 0;
        targetPosition = slideTarget;
        currentPosition = horizontalMotor.getCurrentPosition()*CMPerTick;

        double distanceToTarget = targetPosition - currentPosition;

        double halfwayDistance = targetPosition / 2;
        double newAccelDistance = acceldistance;

        int decelCounter = 0;

        double baseMotorVelocity = (maxVelocity) * 0.15;

        if (acceldistance > halfwayDistance){
            newAccelDistance = halfwayDistance;
        }

        double newMaxVelocity = Math.sqrt(2 * maxAccel * newAccelDistance);

        System.out.println("acceleration_distance: " + acceldistance);
        System.out.println("newMaxVelocity: " + newMaxVelocity);
        System.out.println("newAccelDistance accel: " + newAccelDistance);

        for (int i = 0; i < Math.abs(targetPosition - currentPosition); i++) {
            double targetVelocity;

            if (newAccelDistance > i) {

                int range = (int) Math.abs(newAccelDistance - i);

                double AccelSlope = (double) range / Math.abs(newAccelDistance) * 100;

                AccelSlope = ((100 - AccelSlope) * 0.01);

                targetVelocity = (newMaxVelocity * AccelSlope) + baseMotorVelocity;

                if(targetVelocity != 0){
                    slidetime += Math.abs((1 / targetVelocity) * 1000);
                }

                time.add(slidetime);

                System.out.println("targetVelocity accel: " + targetVelocity);

            }else if (i + newAccelDistance > Math.abs(targetPosition - currentPosition)) {

                decelCounter++;

                int range = (int) Math.abs(newAccelDistance - decelCounter);

                double DeccelSlope = (double) range / Math.abs(newAccelDistance) * 100;

                DeccelSlope = DeccelSlope * 0.01;

                targetVelocity = (newMaxVelocity * DeccelSlope) + baseMotorVelocity;

                positions.add((double) i+1);

                System.out.println("targetVelocity dccel: " + targetVelocity);

            } else {

                targetVelocity = newMaxVelocity;

                positions.add((double) i+1);

                System.out.println("targetVelocity: " + targetVelocity);

            }

            motionProfile.add(targetVelocity);

        }

        System.out.println("slide time: " + time.size());
        System.out.println("motion profile: " + motionProfile.size());
        System.out.println("positions size: " + positions.size());
        for (int i = 0; i < positions.size(); i++){
            System.out.println("positions profile: " + positions.get(i));
        }

    }

    public double getRailPosition() {
        return currentRailPosition;
    }

    public void setRailTargetPosition(double targetPosition) {
        this.railTargetPosition = targetPosition;
        updateRailPosition();
    }

    public void updateRailPosition(){

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
            currentRailPosition += deltaPosition*cmPerDegree;
        }

//        runToPosition();

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

    public gripper getGripperState() {
        return gripperState;
    }

    public void setGripperState(gripper gripperState) {
        this.gripperState = gripperState;
    }

    public Nest getNestState() {
        return nestState;
    }

    public void setNestState(Nest nestState) {
        this.nestState = nestState;
    }

    public fourBar getCollectionState() {
        return collectionState;
    }

    public void setCollectionState(fourBar collectionState) {
        this.collectionState = collectionState;
    }

    public slideState getSlidesState() {
        return slidesState;
    }

    public void setSlidesState(slideState slidesState) {
        this.slidesState = slidesState;
    }

}
