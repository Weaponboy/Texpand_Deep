package dev.weaponboy.command_library.Hardware;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

public class motionProfile {

    double targetPosition = 0;
    double currentPosition = 0;

    double maxAcceleration;
    double maxVelocity;
    double accelDistance;

    ArrayList<Double> motionProfile = new ArrayList<>();
    ArrayList<Double> positions = new ArrayList<>();
    ArrayList<Double> time = new ArrayList<>();

    ElapsedTime currentTime = new ElapsedTime();

    int lastIndex;
    double slideTime;
    double velocityToMotorPower;

    public int maxSlideHeight;
    public double maxSlideHeightCM;

    public double CMPerTick;
    double holdingMotorPower;

    public boolean isSlideRunning() {
        return slideRunning;
    }

    boolean slideRunning = false;

    boolean vertical = false;

    public void isVertical(boolean vertical) {
        this.vertical = vertical;
    }

    public motionProfile(double maxAcceleration, double maxVelocity, double maxSlideHeightCM, int maxSlideHeightTicks, double baseMotorPower){
        this.maxAcceleration = maxAcceleration;
        this.maxVelocity = maxVelocity;
        this.maxSlideHeightCM = maxSlideHeightCM;
        this.maxSlideHeight = maxSlideHeightTicks;
        this.holdingMotorPower = baseMotorPower;

        accelDistance = (maxVelocity * maxVelocity) / (maxAcceleration*2);
        CMPerTick = maxSlideHeightCM / maxSlideHeight;
        velocityToMotorPower = 1/maxVelocity;
    }

    public void generateMotionProfile (double slideTarget, double currentPosition){

        targetPosition = slideTarget;
        this.currentPosition = currentPosition*CMPerTick;

        if (vertical){
            vertical();
        }else {
            horizontal();
        }

        System.out.println("motion profile size" + motionProfile.size());

        for (int i = 0; i < motionProfile.size()-1; i++){
            System.out.println("motion profile" + motionProfile.get(i));
        }

    }

    public double followProfile(double currentPosition){
        this.currentPosition = (currentPosition*CMPerTick);

        if (!slideRunning){
            currentTime.reset();
            lastIndex = 0;
            slideRunning = true;
        }

        if (lastIndex >= time.size()){
            if (targetPosition > this.currentPosition){
                while (positions.get(lastIndex-time.size()) < this.currentPosition){
                    lastIndex++;
                }
            } else if (targetPosition < this.currentPosition) {
                if (positions.get(lastIndex-time.size()) > this.currentPosition){
                    lastIndex++;
                }
            }
        }else {

            if (time.get(lastIndex) < currentTime.milliseconds()) {
                lastIndex++;
            }

        }

        double targetVelocity;
        double targetMotorPower;

        if (lastIndex < motionProfile.size()-2){
            targetVelocity = motionProfile.get(lastIndex);
            targetMotorPower = targetVelocity*velocityToMotorPower;
        }else {
            if(targetPosition == 0){
                targetMotorPower = -1;
            }else {
                targetMotorPower = 0;
            }
            slideRunning = false;
        }


//        slideRunning = currentPosition > targetPosition;
//
//        if (lastIndex >= motionProfile.size()-2){
//
//        }

        return targetMotorPower;

    }

    public void vertical(){

        time.clear();
        motionProfile.clear();
        positions.clear();
        slideTime = 0;

        double halfwayDistance = targetPosition / 2;
        double newAccelDistance = accelDistance;

        int decelCounter = 0;

        double baseMotorVelocity = (maxVelocity) * holdingMotorPower;

        if (accelDistance > halfwayDistance){
            newAccelDistance = halfwayDistance;
        }

        double newMaxVelocity = Math.sqrt(2 * maxAcceleration * newAccelDistance);

        for (int i = 0; i < Math.abs(targetPosition - currentPosition); i++) {
            double targetVelocity;

            if (newAccelDistance > i && currentPosition < targetPosition) {

                int range = (int) Math.abs(newAccelDistance - i);

                double AccelSlope = (double) range / Math.abs(newAccelDistance) * 100;

                AccelSlope = ((100 - AccelSlope) * 0.01);

                targetVelocity = (newMaxVelocity * AccelSlope) + baseMotorVelocity;

                if (targetVelocity != 0) {
                    slideTime += Math.abs((1 / targetVelocity) * 1000);
                }

                time.add(slideTime);

            } else if (i + newAccelDistance > Math.abs(targetPosition - currentPosition) && currentPosition > targetPosition) {

                decelCounter++;

                int range = (int) Math.abs(newAccelDistance - decelCounter);

                double DeccelSlope = (double) range / Math.abs(newAccelDistance) * 100;

                DeccelSlope = DeccelSlope * 0.01;

                targetVelocity = (newMaxVelocity * DeccelSlope) + baseMotorVelocity;

                positions.add((double) i + 1);

            } else {

                targetVelocity = newMaxVelocity;

                positions.add((double) i + 1);

            }

            motionProfile.add(targetVelocity);
        }
    }

    public void horizontal(){

        time.clear();
        motionProfile.clear();
        positions.clear();
        slideTime = 0;

        double halfwayDistance = targetPosition / 2;
        double newAccelDistance = accelDistance;

        int decelCounter = 0;

        double baseMotorVelocity = (maxVelocity) * holdingMotorPower;

        if (accelDistance > halfwayDistance){
            newAccelDistance = halfwayDistance;
        }

        double newMaxVelocity = Math.sqrt(2 * maxAcceleration * newAccelDistance);

        for (int i = 0; i < Math.abs(targetPosition - currentPosition); i++) {
            double targetVelocity;

            if (newAccelDistance > i) {

                int range = (int) Math.abs(newAccelDistance - i);

                double AccelSlope = (double) range / Math.abs(newAccelDistance) * 100;

                AccelSlope = ((100 - AccelSlope) * 0.01);

                targetVelocity = (newMaxVelocity * AccelSlope) + baseMotorVelocity;

                if (targetVelocity != 0) {
                    slideTime += Math.abs((1 / targetVelocity) * 1000);
                }

                time.add(slideTime);

            } else if (i + newAccelDistance > Math.abs(targetPosition - currentPosition)) {

                decelCounter++;

                int range = (int) Math.abs(newAccelDistance - decelCounter);

                double DeccelSlope = (double) range / Math.abs(newAccelDistance) * 100;

                DeccelSlope = DeccelSlope * 0.01;

                targetVelocity = (newMaxVelocity * DeccelSlope) + baseMotorVelocity;

                positions.add((double) i + 1);

            } else {

                targetVelocity = newMaxVelocity;

                positions.add((double) i + 1);

            }

            motionProfile.add(targetVelocity);
        }
    }

}
