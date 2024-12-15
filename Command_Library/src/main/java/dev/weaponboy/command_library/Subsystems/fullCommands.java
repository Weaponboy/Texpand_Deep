package dev.weaponboy.command_library.Subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;

import dev.weaponboy.command_library.CommandLibrary.Commands.LambdaCommand;
import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;
import dev.weaponboy.command_library.CommandLibrary.Subsystem.SubSystem;

public class fullCommands extends SubSystem {

    public DriveBase driveBase;
    public Collection collection;
    public Delivery delivery;
    public Odometry odometry;

    ElapsedTime commandTimer = new ElapsedTime();

    enum Obs_Collect{
        waiting,
        positionToGrab,
        drop,
        collect
    }

    Obs_Collect obsCollect = Obs_Collect.waiting;

    public fullCommands(OpModeEX opModeEX, Delivery delivery, Collection collection, Odometry odometry, DriveBase drivebase){
        registerSubsystem(opModeEX, defaultCommand);
        this.driveBase = drivebase;
        this.collection = collection;
        this.odometry = odometry;
        this.delivery = delivery;
    }

    @Override
    public void init() {

    }

    @Override
    public void execute() {
        executeEX();
    }

    LambdaCommand defaultCommand = new LambdaCommand(
            () -> {},
            () -> {},
            () -> true
    );

//    Command obs_Collect = new LambdaCommand(
//        () -> {
//            obsCollect = Obs_Collect.extendSlides;
//        },
//        () -> {
//
//            if (obsCollect == Obs_Collect.extendSlides && collection.getSlideTarget() < 10){
//                collection.setSlideTarget(34.5);
//            }else if (obsCollect == Obs_Collect.extendSlides && collection.getSlidePositionCM() > 34){
//                obsCollect = Obs_Collect.flipArm;
//                collection.setRailTargetPosition(18);
//                collection.ChamberCollect.execute();
//                commandTimer.reset();
//            }else if (obsCollect == Obs_Collect.flipArm && commandTimer.milliseconds() > 550 && commandTimer.milliseconds() < 700){
//                collection.setClawsState(Collection.clawState.drop);
//                collection.setRailTargetPosition(13);
//                commandTimer.reset();
//                obsCollect = Obs_Collect.drop;
//            }else if (obsCollect == Obs_Collect.drop && commandTimer.milliseconds() > 700) {
//                obsCollect = Obs_Collect.waiting;
//                collection.queueCommand(collection.collect);
//                collection.queueCommand(collection.collect);
//            }
//        },
//        () -> obsCollect == Obs_Collect.waiting
//    );

}
