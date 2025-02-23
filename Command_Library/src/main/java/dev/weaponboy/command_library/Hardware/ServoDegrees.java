package dev.weaponboy.command_library.Hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public class ServoDegrees {

    ServoImplEx servo;
    double range;
    double positionConstantConverter;
    double PWMPerDegree;

    double currentPosition = 0;

    public void setOffset(double offset) {
        this.offset = offset;
    }

    double offset = 0;

    CompletableFuture<Boolean> setPositionAsync;
    private ExecutorService executor;

    public void initServo(String servoName, HardwareMap hardwareMap){
        servo = hardwareMap.get(ServoImplEx.class, servoName);
        this.executor = Executors.newFixedThreadPool(2);
    }

    public void setDirection(Servo.Direction direction){
        servo.setDirection(direction);
    }

    public void setRange(PwmControl.PwmRange range, double degrees){
        servo.setPwmRange(range);
        PWMPerDegree = degrees / 2000;
        this.range = (range.usPulseUpper - range.usPulseLower) * PWMPerDegree;
        positionConstantConverter = 1/this.range;
    }

    public void setRange(double degrees){
        this.range = degrees;
        positionConstantConverter = 1/this.range;
    }

    public void setPosition(double position) {

        double changeInPosition = Math.abs((position+offset) - currentPosition);

        if (setPositionAsync == null){
            currentPosition = position+offset;
            setPositionAsync = setPositionAsync(position+offset);
        } else if (setPositionAsync.isDone() && changeInPosition > 2) {
            currentPosition = position+offset;
            setPositionAsync = setPositionAsync(position+offset);
        }

    }

    public double getPosition() {
        return servo.getPosition();
    }

    public double getPositionDegrees() {
        return (servo.getPosition()/positionConstantConverter);
    }

    private CompletableFuture<Boolean> setPositionAsync(double position) {
        return CompletableFuture.supplyAsync(() -> {
            servo.setPosition(position * positionConstantConverter);
            return true;
        }, executor);
    }

    public void disableServo(){
        servo.getController().pwmDisable();
    }

}
