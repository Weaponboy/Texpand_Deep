package dev.weaponboy.command_library.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public class MotorEx {

    DcMotorEx motor;
    private ExecutorService executor;
    double tolerance = 0.005;

    double currentPower;
    int currentPosition;

    public double getTimeCompleted() {
        return timeCompleted;
    }

    double timeCompleted;

    boolean updatePosition;

    CompletableFuture<Boolean> setPowerFuture;
    CompletableFuture<Boolean> getPowerFuture;
    CompletableFuture<Boolean> getPositionFuture;

    public void initMotor(String motorName, HardwareMap hardwareMap){
        motor = hardwareMap.get(DcMotorEx.class, motorName);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.executor = Executors.newFixedThreadPool(2);
    }

    public Double getPower() {
        return currentPower;
    }

    public Integer getCurrentPosition() {
        return motor.getCurrentPosition();
    }

    public void setDirection(DcMotorSimple.Direction direction){
        motor.setDirection(direction);
    }

    public void setMode(DcMotor.RunMode runMode){
        motor.setMode(runMode);
    }

    public void update(double power){

        double powerDelta = Math.abs(power - currentPower);

        if (setPowerFuture == null){
            setPowerFuture = setPowerAsync(power);
        }else if (setPowerFuture.isDone() &&( powerDelta > tolerance || power == 0)){
            setPowerFuture = setPowerAsync(power);
        }

        if (getPositionFuture == null){
            getPositionFuture = getPositionAsync();
            updatePosition = false;
        }else if (getPositionFuture.isDone()  && updatePosition){
            timeCompleted = System.nanoTime();
            getPositionFuture = getPositionAsync();
            updatePosition = false;
        }

    }

    private CompletableFuture<Boolean> getPositionAsync() {
        return CompletableFuture.supplyAsync(() -> {
            currentPosition = motor.getCurrentPosition();
            return true;
        }, executor);
    }

    public int getDouble (){
        return motor.getCurrentPosition();
    }

    private CompletableFuture<Boolean> setPowerAsync(double power) {
        return CompletableFuture.supplyAsync(() -> {
            motor.setPower(power);
            return true;
        }, executor);
    }

    public double getVelocity(){
        return motor.getVelocity();
    }

    public CompletableFuture<Boolean> getSetPowerFuture() {
        return setPowerFuture;
    }

    private void shutdown() {
        executor.shutdown();
    }

    public void updatePosition() {
        this.updatePosition = true;
    }

}
