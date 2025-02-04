package org.firstinspires.ftc.teamcode.Testing.Teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;
import dev.weaponboy.command_library.Hardware.TargetSample;
import dev.weaponboy.command_library.Subsystems.Collection;
import dev.weaponboy.command_library.Subsystems.Delivery;
import dev.weaponboy.nexus_pathing.RobotUtilities.Vector2D;

@TeleOp(name = "Testing_ Targeting", group = "Testing")
public class TestingTargeting extends OpModeEX {

    @Override
    public void initEX() {

//        odometry.startPosition(342.5, 164, 180);

        odometry.startPosition(82.5, 100, 0);
    }

    @Override
    public void loopEX() {

        if (currentGamepad2.back && !lastGamepad2.back){
            delivery.overrideCurrent(true, delivery.stow);
            collection.overrideCurrent(true, collection.stow);
            delivery.runReset();
            delivery.setGripperState(Delivery.gripper.drop);
            collection.targetPositionManuel = new Vector2D(20, 20);
        }

        if (currentGamepad2.left_stick_y < -0.4){
            collection.armEndPointIncrement(0, 0.5, false);
        }

        if (currentGamepad2.left_stick_y > 0.4){
            collection.armEndPointIncrement(0, -0.5, false);
        }

        if (currentGamepad2.left_stick_x < -0.5 && (collection.getFourBarState() == Collection.fourBar.preCollect || collection.getFourBarState() == Collection.fourBar.collect)){
            collection.armEndPointIncrement(-Math.abs(currentGamepad2.left_stick_x*0.5), 0, false);
        }else if (currentGamepad2.left_stick_x > 0.5 && (collection.getFourBarState() == Collection.fourBar.preCollect || collection.getFourBarState() == Collection.fourBar.collect)){
            collection.armEndPointIncrement(Math.abs(currentGamepad2.left_stick_x*0.5), 0, false);
        }

        if(currentGamepad2.x && !lastGamepad2.x) {
            collection.queueCommand(collection.autoCollectGlobal(new TargetSample(new Vector2D(143.5, 97.5), 0)));
        }

    }
}
