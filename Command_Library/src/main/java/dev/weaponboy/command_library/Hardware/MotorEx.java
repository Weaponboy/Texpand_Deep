package dev.weaponboy.command_library.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public class MotorEx {

    DcMotorEx motor;
    private ExecutorService executor;
    double tolerance = 0.05;

    double currentPower;
    int currentPosition;

    double velocity;

    public double getTimeCompleted() {
        return timeCompleted;
    }

    double timeCompleted;

    boolean updatePosition;

    CompletableFuture<Boolean> setPowerFuture;
    CompletableFuture<Boolean> getPowerFuture;
    CompletableFuture<Boolean> getVelocityFuture;

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

    public double getCurrentDraw() {
        return motor.getCurrent(CurrentUnit.MILLIAMPS);
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
        }else if (setPowerFuture.isDone() && (powerDelta > tolerance || power == 0)){
            setPowerFuture = setPowerAsync(power);
        }

//        if (getVelocityFuture == null){
//            getVelocityFuture = getPositionAsync();
//            updatePosition = false;
//        }else if (getVelocityFuture.isDone()  && updatePosition){
//            timeCompleted = System.nanoTime();
//            getVelocityFuture = getPositionAsync();
//            updatePosition = false;
//        }

    }

    private CompletableFuture<Boolean> getPositionAsync() {
        return CompletableFuture.supplyAsync(() -> {
            velocity = motor.getVelocity();
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

    public void updateVelocity() {
        this.updatePosition = true;
    }

}
