package org.firstinspires.ftc.teamcode.Testing.Teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;
import dev.weaponboy.command_library.Subsystems.Collection;

@TeleOp
public class sprint1Teleop extends OpModeEX {

    @Override
    public void initEX() {

    }

    @Override
    public void loopEX() {

        // drive base code
        driveBase.queueCommand(driveBase.drivePowers(gamepad1.right_stick_y,gamepad1.left_stick_x,gamepad1.right_stick_x));

        //collection code
        if (currentGamepad1.start && !lastGamepad1.start && collection.gripperState == Collection.gripper.drop){
            collection.queueCommand(collection.grab);
        }else if (currentGamepad1.start && !lastGamepad1.start && collection.gripperState == Collection.gripper.grab){
            collection.queueCommand(collection.drop);
        }

//        if (currentGamepad1.)

    }
}