package dev.weaponboy.command_library.Hardware;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

public class MotionProfile {

    double targetPosition = 50;
    double currentPosition = 0;

    double maxAcceleration = 1400;
    double maxVelocity = 140;
    double accelDistance = (maxVelocity * maxVelocity) / (maxAcceleration*2);

    ArrayList<Double> motionProfile = new ArrayList<>();
    ArrayList<Double> positions = new ArrayList<>();
    ArrayList<Double> time = new ArrayList<>();

    ElapsedTime currentTime = new ElapsedTime();

    int lastIndex = 0;
    double slideTime;
    double velocityToMotorPower = 1/maxVelocity;

    public final int maxSlideHeight = 1720;
    public final int maxSlideHeightCM = 53;
    private final int ticksPerCm = maxSlideHeight / maxSlideHeightCM;
    public final double CMPerTick = (double) maxSlideHeightCM / maxSlideHeight;

    public MotionProfile(double maxAcceleration, double maxVelocity){
        this.maxAcceleration = maxAcceleration;
        this.maxVelocity = maxVelocity;
    }



}
