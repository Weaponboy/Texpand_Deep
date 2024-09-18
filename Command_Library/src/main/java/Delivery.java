import dev.weaponboy.command_library.CommandLibrary.Subsystem.SubSystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

import dev.weaponboy.command_library.CommandLibrary.Commands.LambdaCommand;

public class Delivery extends SubSystem {
    DcMotor slidemMotor;
    DcMotor slideMotor;
    double test = 0;
    double targetPosition = 50;
    int currentPosition = 0;
    double maxAccel = 380;
    double maxVelocity = 450;
    double acceldistance = maxAccel/maxVelocity;
    ArrayList<Double> motionprofile = new ArrayList<>();
    ArrayList<Double> time = new ArrayList<>();
    ElapsedTime currentTime = new ElapsedTime();
    int lastIndex = 0;
    double slidetime;
    double veloToMotorPower = 1/maxVelocity;

    public LambdaCommand deliveryeup = new LambdaCommand(
            () -> {
                slidemMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                slidemMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            },
            () -> {
                slidemMotor.setPower(0.5);

            },

            () -> slidemMotor.getCurrentPosition() > 2000

    );
    public LambdaCommand followMotionPro = new LambdaCommand(
            () -> {
                currentTime.reset();



            },
            ()-> {
                while (time.get(lastIndex) < currentTime.milliseconds()){
                    lastIndex++;

                }
                double targetVelocity = motionprofile.get(lastIndex);
                double targetMotorPower = targetVelocity*veloToMotorPower;
                slidemMotor.setPower(targetMotorPower);




            },
            ()->

            slidetime > currentTime.milliseconds()



    );

    @Override
    public void init() {


    }

    @Override
    public void execute() {

    }
    public void genProfile (double slideTarget){

                    System.out.println("init Second");
                    double test = 0;
                    slidetime = 0;
                    targetPosition = slideTarget;
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

                            slidetime += (1 / targetVelocity) * 1000;

                        }
                        if (i + newAccelDistance > Math.abs(targetPosition - currentPosition)) {

                            decelCounter++;

                            int range = (int) Math.abs(newAccelDistance - decelCounter);

                            double DeccelSlope = (double) range / Math.abs(newAccelDistance) * 100;

                            DeccelSlope = DeccelSlope * 0.01;

                            targetVelocity = newMaxVelocity * DeccelSlope;

                            slidetime += (1 / targetVelocity) * 1000;

                        } else {
                            targetVelocity = newMaxVelocity;

                            slidetime += (1 / targetVelocity) * 1000;

                        }
                        motionprofile.add(targetVelocity);
                        time.add(slidetime);
                    }
                }
            }









