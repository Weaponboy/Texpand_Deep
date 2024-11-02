package dev.weaponboy.command_library.Hardware;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

public class motionProfile {

    double targetPosition = 0;
    double currentPosition = 0;

    double startPosition = 0;

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
        startPosition = this.currentPosition;

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

        if (lastIndex >= time.size()-1){
            if (lastIndex - time.size() < 0){
                lastIndex++;
            }

            if (targetPosition > this.startPosition){
                System.out.println("in first if: " + lastIndex);
                while (positions.get(lastIndex-time.size()) < this.currentPosition){
                    System.out.println("in while: " + lastIndex);
                    if (lastIndex < motionProfile.size()-1){
                        lastIndex++;
                        System.out.println("increment: " + lastIndex);
                    }else {
                        break;
                    }
                }
            } else if (targetPosition < this.startPosition) {
                while (positions.get(lastIndex-time.size()) > this.currentPosition){
                    if (lastIndex < motionProfile.size()-1){
                        lastIndex++;
                    }else {
                        break;
                    }
                }
            }


        }else {

            while (time.get(lastIndex) < currentTime.milliseconds()) {
                if (lastIndex < time.size()-1){
                    lastIndex++;
                }else {
                    break;
                }
            }

        }

        double targetVelocity;
        double targetMotorPower;

        double deadZone = 1;

        if (vertical && targetPosition != 0){
            deadZone = 4;
        }

        if (lastIndex < motionProfile.size()-deadZone){
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

        System.out.println("lastIndex: " + lastIndex);
        System.out.println("motionProfile.size()-2: " + (motionProfile.size()-2));

        return targetMotorPower;

    }

    public double followProfile(double currentPosition, double currentVelo){
        this.currentPosition = (currentPosition*CMPerTick);
        currentVelo = currentVelo*CMPerTick;

        if (!slideRunning){
            currentTime.reset();
            lastIndex = 0;
            slideRunning = true;
        }

        if (lastIndex >= time.size()){
            if (targetPosition > this.startPosition){
                System.out.println("in first if: " + lastIndex);
                while (positions.get(lastIndex-time.size()) < this.currentPosition){
                    System.out.println("in while: " + lastIndex);
                    if (lastIndex < motionProfile.size()-1){
                        lastIndex++;
                        System.out.println("increment: " + lastIndex);
                    }else {
                        break;
                    }
                }
            } else if (targetPosition < this.startPosition) {
                while (positions.get(lastIndex-time.size()) > this.currentPosition){
                    if (lastIndex < motionProfile.size()-1){
                        lastIndex++;
                    }else {
                        break;
                    }
                }
            }
        }else {

            if (time.get(lastIndex) < currentTime.milliseconds()) {
                lastIndex++;
            }

        }

        double targetVelocity;
        double targetMotorPower;

        double deadZone = 2;

        if (vertical && targetPosition != 0){
            deadZone = 6;
        }

        if (lastIndex < motionProfile.size()-deadZone){
            targetVelocity = motionProfile.get(lastIndex);
            double veloDef = targetVelocity - currentVelo;
            if (!(targetPosition == 0)){
                targetVelocity += veloDef;
            }
            targetMotorPower = targetVelocity*velocityToMotorPower;
        }else {
            if(targetPosition == 0){
                targetMotorPower = -1;
            }else {
                targetMotorPower = 0;
            }
            slideRunning = false;
        }

        System.out.println("lastIndex: " + lastIndex);
        System.out.println("motionProfile.size()-2: " + (motionProfile.size()-2));

        return targetMotorPower;
    }

    public void vertical(){

        time.clear();
        motionProfile.clear();
        positions.clear();
        slideTime = 0;

        double halfwayDistance = Math.abs(targetPosition - currentPosition) / 2;
        double velocityHalf = (targetPosition - currentPosition) / 2;
        double newAccelDistance = accelDistance;

        int decelCounter = 0;

        double baseMotorVelocity = (maxVelocity) * holdingMotorPower;

        if (accelDistance > halfwayDistance){
            newAccelDistance = halfwayDistance;
        }

        double newMaxVelocity = Math.sqrt(2 * maxAcceleration * newAccelDistance);

        if (velocityHalf < 0){
            newMaxVelocity = -newMaxVelocity;
            baseMotorVelocity = -baseMotorVelocity;
        }

        System.out.println("newMaxVelocity " + newMaxVelocity);
        System.out.println("baseMotorVelocity " + baseMotorVelocity);
        System.out.println("newAccelDistance " + newAccelDistance);
        System.out.println("Math.abs(targetPosition - currentPosition) " + Math.abs(targetPosition - currentPosition));

        for (int i = 0; i < Math.abs(targetPosition - currentPosition); i++) {
            double targetVelocity;

            if (Math.abs(newAccelDistance) > i && targetPosition > startPosition) {

                int range = (int) Math.abs(newAccelDistance - i);

                double AccelSlope = (double) range / Math.abs(newAccelDistance) * 100;

                AccelSlope = ((100 - AccelSlope) * 0.01);

                targetVelocity = (newMaxVelocity * AccelSlope) + baseMotorVelocity;

                if (targetVelocity != 0) {
                    slideTime += (1 / Math.abs(targetVelocity)) * 1000;
                }

                time.add(slideTime);

                System.out.println("targetVelocity accel" + targetVelocity);
                System.out.println("time accel" + slideTime);


            }else {

                targetVelocity = newMaxVelocity;

                if (velocityHalf < 0){
                    positions.add(currentPosition - i+1);
                    System.out.println("position normal" + (currentPosition - i+1));
                }else {
                    positions.add(currentPosition + i+1);
                    System.out.println("position normal" + (currentPosition + i+1));
                }

                System.out.println("targetVelocity normal" + targetVelocity);
//                System.out.println("position normal" + (double) i + 1);

            }

//            else if (i + Math.abs(newAccelDistance) > Math.abs(targetPosition - currentPosition) && false) {
//
//                decelCounter++;
//
//                int range = (int) Math.abs(newAccelDistance - decelCounter);
//
//                double DeccelSlope = (double) range / Math.abs(newAccelDistance) * 100;
//
//                DeccelSlope = DeccelSlope * 0.01;
//
//                if (targetPosition == 0){
//                    targetVelocity = (newMaxVelocity * DeccelSlope) + baseMotorVelocity;
//                }else {
//                    targetVelocity = (newMaxVelocity * DeccelSlope);
//                }
//
//                System.out.println("targetVelocity decel" + targetVelocity);
//
//                if (velocityHalf < 0){
//                    positions.add(currentPosition - i+1);
//                    System.out.println("position decel" + (currentPosition - i+1));
//                }else {
//                    positions.add(currentPosition + i+1);
//                    System.out.println("position decel" + (currentPosition + i+1));
//                }
//
//            }


            motionProfile.add(targetVelocity);
        }
    }

    public void horizontal(){

        time.clear();
        motionProfile.clear();
        positions.clear();
        slideTime = 0;

        double halfwayDistance = Math.abs(targetPosition - currentPosition) / 2;
        double velocityHalf = (targetPosition - currentPosition) / 2;
        double newAccelDistance = accelDistance;

        int decelCounter = 0;

        double baseMotorVelocity = (maxVelocity) * holdingMotorPower;

        if (accelDistance > halfwayDistance){
            newAccelDistance = halfwayDistance;
        }

        double newMaxVelocity = Math.sqrt(2 * maxAcceleration * newAccelDistance);

        if (velocityHalf < 0){
            newMaxVelocity = -newMaxVelocity;
            baseMotorVelocity = -baseMotorVelocity;
        }

        System.out.println("newMaxVelocity " + newMaxVelocity);
        System.out.println("baseMotorVelocity " + baseMotorVelocity);
        System.out.println("newAccelDistance " + newAccelDistance);
        System.out.println("Math.abs(targetPosition - currentPosition) " + Math.abs(targetPosition - currentPosition));

        for (int i = 0; i < Math.abs(targetPosition - currentPosition); i++) {
            double targetVelocity;

            if (Math.abs(newAccelDistance) > i) {

                int range = (int) Math.abs(newAccelDistance - i);

                double AccelSlope = (double) range / Math.abs(newAccelDistance) * 100;

                AccelSlope = ((100 - AccelSlope) * 0.01);

                targetVelocity = (newMaxVelocity * AccelSlope) + baseMotorVelocity;

                if (targetVelocity != 0) {
                    slideTime += (1 / Math.abs(targetVelocity)) * 1000;
                }

                time.add(slideTime);

                System.out.println("targetVelocity accel" + targetVelocity);
                System.out.println("time accel" + slideTime);


            } else if (i + Math.abs(newAccelDistance) > Math.abs(targetPosition - currentPosition)) {

                decelCounter++;

                int range = (int) Math.abs(newAccelDistance - decelCounter);

                double DeccelSlope = (double) range / Math.abs(newAccelDistance) * 100;

                DeccelSlope = DeccelSlope * 0.01;

                if (targetPosition == 0){
                    targetVelocity = (newMaxVelocity * DeccelSlope) + baseMotorVelocity;
                }else {
                    targetVelocity = (newMaxVelocity * DeccelSlope);
                }

                System.out.println("targetVelocity decel" + targetVelocity);

                if (velocityHalf < 0){
                    positions.add(currentPosition - i+1);
                    System.out.println("position decel" + (currentPosition - i+1));
                }else {
                    positions.add(currentPosition + i+1);
                    System.out.println("position decel" + (currentPosition + i+1));
                }

            } else {

                targetVelocity = newMaxVelocity;

                if (velocityHalf < 0){
                    positions.add(currentPosition - i+1);
                    System.out.println("position normal" + (currentPosition - i+1));
                }else {
                    positions.add(currentPosition + i+1);
                    System.out.println("position normal" + (currentPosition + i+1));
                }

                System.out.println("targetVelocity normal" + targetVelocity);
//                System.out.println("position normal" + (double) i + 1);

            }

            motionProfile.add(targetVelocity);
        }
    }

}
