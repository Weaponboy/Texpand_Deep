package dev.weaponboy.command_library.Hardware;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public class MotorEx {

    DcMotorEx motor;
    private ExecutorService executor;

    double currentPower;
    int currentPosition;
    double current;

    public CompletableFuture<Boolean> getSetPowerFuture() {
        return setPowerFuture;
    }

    CompletableFuture<Boolean> setPowerFuture;
    CompletableFuture<Boolean> getPowerFuture;

    public void initMotor(String motorName, HardwareMap hardwareMap){
        motor = hardwareMap.get(DcMotorEx.class, motorName);
        this.executor = Executors.newFixedThreadPool(2);
    }

    public void update(Double power){
//        if (positionFuture == null){
//            positionFuture = getPositionAsync();
//        }else if (!positionFuture.isDone()){
//            positionFuture = getPositionAsync();
//        }

        setPowerFuture = setPowerAsync(power);

//        if (setPowerFuture == null){
//
//        }else if (setPowerFuture.isDone()){
//            setPowerFuture = setPowerAsync(power);
//        }

//        if (getPowerFuture == null){
//            getPowerFuture = getPowerAsync();
//        }else if (getPowerFuture.isDone()){
//            getPowerFuture = getPowerAsync();
//        }

    }

    public Double getPower() {
        return currentPower;
    }

    public Integer getCurrentPosition() {
        return currentPosition;
    }

    private CompletableFuture<Boolean> getPositionAsync() {
        return CompletableFuture.supplyAsync(() -> {
            currentPosition = motor.getCurrentPosition();
            return true;
        }, executor);
    }

    private CompletableFuture<Boolean> setPowerAsync(double power) {
        return CompletableFuture.supplyAsync(() -> {
            motor.setPower(power);
            return true;
        }, executor);
    }

    private CompletableFuture<Boolean> getPowerAsync() {
        return CompletableFuture.supplyAsync(() -> {
            currentPower = motor.getPower();
            return true;
        }, executor);
    }

    public void setPower(Double power){
        motor.setPower(power);
    }

    public double getNormalPower(){
        return motor.getPower();
    }

    private void shutdown() {
        executor.shutdown();
    }

}
