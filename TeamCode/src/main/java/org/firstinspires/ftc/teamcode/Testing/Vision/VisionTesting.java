package org.firstinspires.ftc.teamcode.Testing.Vision;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;
import dev.weaponboy.command_library.Subsystems.Delivery;
import dev.weaponboy.nexus_pathing.RobotUtilities.Vector2D;

@TeleOp(name = "Vision_Testing", group = "Testing")
public class VisionTesting extends OpModeEX {

    boolean busyDetecting = false;
    ElapsedTime detectionTimer = new ElapsedTime();
    int counter = 0;

    @Override
    public void initEX() {
        odometry.startPosition(82.5,100,0);
    }

    @Override
    public void loopEX() {

        if (currentGamepad1.back && !lastGamepad1.back){
            delivery.overrideCurrent(true, delivery.stow);
            collection.overrideCurrent(true, collection.stow);
            delivery.runReset();
            delivery.setGripperState(Delivery.gripper.drop);
            collection.targetPositionManuel = new Vector2D(15, 20);
            collection.armEndPointIncrement(0,0, false);
        }

        if(((currentGamepad1.dpad_left && !lastGamepad1.dpad_left)) && delivery.fourbarState != Delivery.fourBarState.preClip){

            delivery.queueCommand(delivery.cameraScan);

            collection.queueCommand(collection.visionScan);

            collection.targetPositionManuel = new Vector2D(6, 20);
            collection.armEndPointIncrement(16, -12, false);

            limelight.setReturningData(true);
            limelight.setGettingResults(true);

        }

        if (((currentGamepad1.dpad_left && !lastGamepad1.dpad_left)) && delivery.getSlidePositionCM() > 15 && delivery.getGripperState() != Delivery.gripper.grab){
            busyDetecting = true;
            detectionTimer.reset();
            counter = 0;
            delivery.griperRotateSev.setPosition(90);
        }

        if (busyDetecting && detectionTimer.milliseconds() > (50*counter) && counter < 20){

            counter++;

            if (limelight.getTargetPoint() != null){

                collection.queueCommand(collection.autoCollectGlobal(limelight.returnPointToCollect()));

                delivery.overrideCurrent(true, delivery.stow);
                delivery.runReset();

                busyDetecting = false;
                counter = 40;
            }

        } else if (busyDetecting && detectionTimer.milliseconds() > (50*counter) && counter > 20) {

            delivery.overrideCurrent(true, delivery.stow);
            delivery.runReset();

            busyDetecting = false;
        }

        telemetry.addData("collection is busy", collection.getCurrentCommand() == collection.defaultCommand);
        if (limelight.getTargetPoint() != null){
            telemetry.addData("limelight results X", limelight.getTargetPoint().getTargetPoint().getX());
            telemetry.addData("limelight results Y", limelight.getTargetPoint().getTargetPoint().getY());
            telemetry.addData("limelight results angle", limelight.getTargetPoint().getAngle());
        }
        telemetry.update();

    }
}
