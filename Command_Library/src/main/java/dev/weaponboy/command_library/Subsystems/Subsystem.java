package dev.weaponboy.command_library.Subsystems;

import java.util.ArrayList;

import dev.weaponboy.command_library.CommandLibrary.Commands.LambdaCommand;
import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;
import dev.weaponboy.command_library.CommandLibrary.Subsystem.SubSystem;

public class Subsystem extends SubSystem {

    int targetPosition = 0;
    int currentPosition = 0;
    double maxAccel = 380;
    double maxVelocity = 450;
    double accelDistance = maxVelocity/maxAccel;

    ArrayList<Double> motionProfile = new ArrayList<>();
    double slideTime;

    public Subsystem(OpModeEX opModeEX){
        registerSubsystem(opModeEX, defaultCommand);
    }

    @Override
    public void init() {

    }

    @Override
    public void execute() {
        executeEX();
        targetPosition++;
    }

    LambdaCommand defaultCommand = new LambdaCommand(
            () -> System.out.println("init"),
            () -> System.out.println("execute"),
            () -> targetPosition > 5
    );

    public LambdaCommand genProfile = new LambdaCommand(
            () -> {
                slideTime = 0;
                double halfwayDistance = targetPosition/2;
                double newAccelDistance = accelDistance;
                int decelCounter = 0;

                if (accelDistance > halfwayDistance){
                    newAccelDistance = halfwayDistance;
                }

                double newMaxVelocity = accelDistance*maxAccel;

                for (int i = 0; i < Math.abs(targetPosition-currentPosition); i++){
                    double targetVelocity;

                    if (newAccelDistance > i){

                        int range = (int) Math.abs(newAccelDistance - i);

                        double AccelSlope = (double) range / (double) Math.abs(newAccelDistance) * 100;

                        AccelSlope = 100 - (AccelSlope*0.01);

                        targetVelocity = newMaxVelocity * AccelSlope;

                        slideTime += (1/targetVelocity)*1000;

                    }if (i+newAccelDistance > Math.abs(targetPosition-currentPosition)){

                        decelCounter++;

                        int range = (int) Math.abs(newAccelDistance - decelCounter);

                        double DeccelSlope = (double) range / Math.abs(newAccelDistance) * 100;

                        DeccelSlope = DeccelSlope*0.01;

                        targetVelocity = newMaxVelocity * DeccelSlope;

                        slideTime += (1/targetVelocity)*1000;

                    }else {
                        targetVelocity = newMaxVelocity;

                        slideTime += (1/targetVelocity)*1000;
                    }

                    motionProfile.add(targetVelocity);

                }


            },
            () -> System.out.println("execute Second"),
            () -> true
    );

    public LambdaCommand defaultCommandThird = new LambdaCommand(
            () -> {
                System.out.println("init Third");
                targetPosition = 0;
            },
            () -> System.out.println("execute Third"),
            () -> targetPosition > 5
    );

}
