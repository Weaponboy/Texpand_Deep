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

    public boolean isScanning = false;
    public boolean closeFirst = true;

    public boolean resultIsValid = false;

    int isScanningInt = 1;
    int closeFirstInt = 1;

    public LLResult result;

    @Override
    public void init() {
        limelight = getOpModeEX().hardwareMap.get(Limelight3A.class, "limelight");

        limelight.setPollRateHz(100);

        limelight.pipelineSwitch(0);
    }

    public void onStart(){
        limelight.start();
    }

    @Override
    public void execute() {
        result = limelight.getLatestResult();

        double[] pythonOutput = result.getPythonOutput();
        targetPoints.clear();

        if (pythonOutput[0] != 0 && pythonOutput[1] != 0){
            targetPoints.add(new TargetSample(new Vector2D(pythonOutput[0], pythonOutput[1]), pythonOutput[2]));
        }

        if (pythonOutput[3] != 0 && pythonOutput[4] != 0){
            targetPoints.add(new TargetSample(new Vector2D(pythonOutput[3], pythonOutput[4]), pythonOutput[5]));
        }

    }

    public void updatePythonInputs(double X, double Y, double Heading, double SlideCM){
        double[] inputs = {isScanningInt, closeFirstInt, X, Y, Heading, SlideCM};
        limelight.updatePythonInputs(inputs);
    }

    public TargetSample getTargetPoint(){

        if (!targetPoints.isEmpty()){
            TargetSample targetSample = targetPoints.get(0);
//            targetPoints.remove(0);
            return targetSample;
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
