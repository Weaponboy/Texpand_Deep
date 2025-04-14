package dev.weaponboy.command_library.Subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

import dev.weaponboy.command_library.CommandLibrary.Commands.LambdaCommand;
import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;
import dev.weaponboy.command_library.CommandLibrary.Subsystem.SubSystem;
import dev.weaponboy.command_library.Hardware.TargetSample;
import dev.weaponboy.nexus_pathing.RobotUtilities.Vector2D;

public class Limelight extends SubSystem {

    private Limelight3A limelight;

    public Limelight(OpModeEX opModeEX){
        registerSubsystem(opModeEX, defaultCommand);
    }

    ArrayList<TargetSample> targetPoints = new ArrayList<>();

    public void setGettingResults(boolean gettingResults) {
        isGettingResults = gettingResults;
    }

    boolean isGettingResults = true;
    double offset = 0;

    public ElapsedTime MovementTimer = new ElapsedTime();

    public double getCurrentTime() {
        return currentTime;
    }

    double currentTime = 0;

    public double OdometryTime = 0;
    public double limelight_elapsed_ms = 0;

    public void setDetectionColor(boolean detectingRed) {
        this.detectingRed = detectingRed;
        redDetection = detectingRed ? 1 : 0;
    }

    public boolean getAllianceSamples(){
        return detectingRed;
    }

    double odoTime = 0;

    public void setOdoTime(){
        odoTime = MovementTimer.milliseconds() - 40;
    }

    public enum color {
        red,
        yellow,
        blue
    }

    public void setAuto(boolean auto) {
        this.auto = auto;
    }

    boolean auto = false;

    public color getTargetColor() {
        return targetColor;
    }

    public void setTargetColor(color targetColor) {
        this.targetColor = targetColor;
        switch (targetColor){
            case red:
                limelight.pipelineSwitch(2);
                break;
            case yellow:
                if (auto) {
                    limelight.pipelineSwitch(1);
                } else {
                    limelight.pipelineSwitch(0);
                }
                break;
            case blue:
                limelight.pipelineSwitch(3);
                break;
            default:
        }
    }

    private color targetColor = color.yellow;

    public boolean detectingRed = true;

    public boolean isAllianceColor() {
        return allianceColor;
    }

    public void collectColoredSamples (boolean collecting) {
        this.allianceColor = collecting;
        allianceColorInt = allianceColor ? 1 : 0;
    }

    public boolean allianceColor = true;

    int redDetection = 1;
    int allianceColorInt = 0;

    public void setReturningData(boolean returningData) {
        this.returningData = returningData;

        if (returningData){
            limelight.start();
        }else {
            limelight.pause();
        }
    }

    boolean returningData = false;

    public LLResult result;

    @Override
    public void init() {
        limelight = getOpModeEX().hardwareMap.get(Limelight3A.class, "limelight");

        limelight.setPollRateHz(100);

        limelight.pipelineSwitch(0);
    }

    public void shutDown(){
        limelight.close();
    }

    public void onStart(){
        limelight.start();
    }

    @Override
    public void execute() {

        if (returningData){

            result = limelight.getLatestResult();

            if (result != null && isGettingResults){

                double[] pythonOutput = result.getPythonOutput();

                targetPoints.clear();

                if (pythonOutput[0] != 0 && pythonOutput[1] != 0){
                    targetPoints.add(new TargetSample(new Vector2D(pythonOutput[0], pythonOutput[1]), pythonOutput[2]));
                }

                if (pythonOutput[3] != 0 && pythonOutput[4] != 0){
                    targetPoints.add(new TargetSample(new Vector2D(pythonOutput[3], pythonOutput[4]), pythonOutput[5]));
                }

//                OdometryTime = pythonOutput[5];
//                limelight_elapsed_ms = pythonOutput[6];
//                currentTime = pythonOutput[7];

//                System.out.println("odo Time output" + OdometryTime);
//                System.out.println("limelight_elapsed_ms" + limelight_elapsed_ms);
//                System.out.println("elapsed time" + currentTime);
            }

        }

//        else {
//            result = limelight.getLatestResult();
//
//            if (result != null){
//                double[] pythonOutput = result.getPythonOutput();
//
////                OdometryTime = pythonOutput[5];
////                limelight_elapsed_ms = pythonOutput[6];
//                currentTime = pythonOutput[7];
//
////                System.out.println("odo Time output" + OdometryTime);
////                System.out.println("limelight_elapsed_ms" + limelight_elapsed_ms);
//                System.out.println("elapsed time" + currentTime);
//            }
//        }

    }

    public void updatePythonInputs(double X, double Y, double Heading, double SlideCM, double getXVelocity, double getYVelocity ){
        double[] inputs = {getXVelocity, redDetection, X, Y, Heading, SlideCM, allianceColorInt, getYVelocity};
//        System.out.println("odo Time" + odoTime);
//        System.out.println("X Velocity" + getXVelocity);
//        System.out.println("Y Velocity" + getYVelocity);
        limelight.updatePythonInputs(inputs);
    }

    public void updateTimeStamp(){
        double[] inputs = {1, 2, 3, 4, 5, 6, 7, 8};
        limelight.updatePythonInputs(inputs);
        MovementTimer.reset();

        currentTime = 0;

        System.out.println("Sent update to Limelight");

//        while (currentTime == 0){
//            result = limelight.getLatestResult();
//
//            System.out.println("In while loop" + MovementTimer.milliseconds());
//
//            if (result != null){
//                double[] pythonOutput = result.getPythonOutput();
//                currentTime = pythonOutput[7];
//                System.out.println("Current time" + currentTime);
//                offset = MovementTimer.milliseconds();
//                System.out.println("offset" + offset);
//            }
//        }

//        double counter = 0;
//
//        while (counter < 100){
//            result = limelight.getLatestResult();
//
//            System.out.println("In while loop" + MovementTimer.milliseconds());
//
//            if (result != null){
//                double[] pythonOutput = result.getPythonOutput();
//                currentTime = pythonOutput[7];
////                System.out.println("Current time" + currentTime);
////                offset = MovementTimer.milliseconds();
////                System.out.println("offset" + offset);
//            }
//
//            System.out.println("Limelight time" + getCurrentTime());
//            System.out.println("Robot time" + getRobotTime());
//
//            counter++;
//        }

    }

    public double getRobotTime(){
        return MovementTimer.milliseconds() - offset;
    }

    public void switchPipeline(int pipelineIndex){
        limelight.pipelineSwitch(pipelineIndex);
    }

    public TargetSample getTargetPoint(){
        if (!targetPoints.isEmpty()){
            return targetPoints.get(0);
        }else {
            return null;
        }

    }

    public TargetSample returnPointToCollect(){

        if (!targetPoints.isEmpty()){
            TargetSample targetSample = targetPoints.get(0);
            targetPoints.remove(0);
            return targetSample;
        }else {
            return null;
        }

    }

    LambdaCommand defaultCommand = new LambdaCommand(
            () -> {},
            () -> {},
            () -> true
    );
}
