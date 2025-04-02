package dev.weaponboy.command_library.Subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

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

    public boolean isGettingResults() {
        return isGettingResults;
    }

    public void setGettingResults(boolean gettingResults) {
        isGettingResults = gettingResults;
    }

    boolean isGettingResults = true;

    public boolean isCloseFirst() {
        return closeFirst;
    }

    public void setCloseFirst(boolean closeFirst) {
        this.closeFirst = closeFirst;
        closeFirstInt = closeFirst ? 1 : 0;
    }

    public boolean isScanning() {
        return isScanning;
    }

    public void setScanning(boolean scanning) {
        isScanning = scanning;
        isScanningInt = isScanning ? 1 : 0;
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

    public boolean isScanning = false;
    public boolean closeFirst = true;

    public boolean isSortHorizontal() {
        return sortHorizontal;
    }

    public void setSortHorizontal(boolean sortHorizontal) {
        this.sortHorizontal = sortHorizontal;
        horInt = sortHorizontal ? 1 : 0;
    }

    public boolean sortHorizontal = true;

    public boolean resultIsValid = false;

    int isScanningInt = 1;
    int closeFirstInt = 1;
    int horInt = 0;

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

        limelight.setPollRateHz(50);

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
            }

        }

    }

    public void updatePythonInputs(double X, double Y, double Heading, double SlideCM, double getXVelocity, double getYVelocity ){
        double[] inputs = {getXVelocity, closeFirstInt, X, Y, Heading, SlideCM, horInt, getYVelocity};
        limelight.updatePythonInputs(inputs);
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
