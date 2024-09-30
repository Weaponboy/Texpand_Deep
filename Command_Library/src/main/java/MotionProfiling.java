import dev.weaponboy.command_library.CommandLibrary.Commands.Command;
import dev.weaponboy.command_library.CommandLibrary.Subsystem.SubSystem;
import java.util.ArrayList;
import dev.weaponboy.command_library.CommandLibrary.Commands.LambdaCommand;
import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;
import dev.weaponboy.command_library.CommandLibrary.Subsystem.SubSystem;

public class MotionProfiling extends SubSystem {
   double test = 0;
    int targetPosition = 50;
    int currentPosition = 0;
    double maxAccel = 380;
    double maxVelocity = 450;
    double acceldistance = maxAccel/maxVelocity;
    ArrayList<Double> motionprofile = new ArrayList<>();
    double slidetime;



public LambdaCommand defaultCommandSecond = new LambdaCommand;

;public LambdaCommand genProfile = new LambdaCommand(
        () -> {
            System.out.println("init Second");
            double test = 0;
            double slideTime = 0;
            double halfwayDistance = targetPosition / 2;
            double newAccelDistance = acceldistance;
            int decelCounter = 0;

            if (acceldistance > halfwayDistance) {
                newAccelDistance = halfwayDistance;
            }

            double newMaxVelocity = acceldistance * maxAccel;

            for (int i = 0; i < Math.abs(targetPosition - currentPosition); i++) {
                double targetVelocity;

            if (newAccelDistance > i) {

                    int range = (int) Math.abs(newAccelDistance - i);

                    double AccelSlope = (double) range / (double) Math.abs(newAccelDistance) * 100;

                    AccelSlope = 100 - (AccelSlope * 0.01);

                    targetVelocity = newMaxVelocity * AccelSlope;

                    slideTime += (1 / targetVelocity) * 1000;

                }
            if (i + newAccelDistance > Math.abs(targetPosition - currentPosition)) {

                    decelCounter++;

                    int range = (int) Math.abs(newAccelDistance - decelCounter);

                    double DeccelSlope = (double) range / Math.abs(newAccelDistance) * 100;

                    DeccelSlope = DeccelSlope * 0.01;

                    targetVelocity = newMaxVelocity * DeccelSlope;

                    slideTime += (1 / targetVelocity) * 1000;

                } else {
                    targetVelocity = newMaxVelocity;

                    slideTime += (1 / targetVelocity) * 1000;
                }
            }
        },


            ()->{

            },
            () -> true;
        );
}










