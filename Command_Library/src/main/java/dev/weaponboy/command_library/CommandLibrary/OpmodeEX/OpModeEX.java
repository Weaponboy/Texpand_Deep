package dev.weaponboy.command_library.CommandLibrary.OpmodeEX;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.IOException;
import java.util.List;

import dev.weaponboy.command_library.CommandLibrary.Subsystem.SubSystem;
import dev.weaponboy.command_library.Subsystems.Collection;
import dev.weaponboy.command_library.Subsystems.Delivery;
import dev.weaponboy.command_library.Subsystems.DriveBase;
import dev.weaponboy.command_library.Subsystems.Hang;
import dev.weaponboy.command_library.Subsystems.Limelight;
import dev.weaponboy.command_library.Subsystems.Odometry;
import dev.weaponboy.nexus_pathing.PathingUtility.RobotPower;

public abstract class OpModeEX extends OpMode {

    public DriveBase driveBase = new DriveBase(this);

    public Collection collection = new Collection(this);

    public Delivery delivery = new Delivery(this);

    public Odometry odometry = new Odometry(this);

    public Limelight limelight = new Limelight(this);

    public Hang hang = new Hang(this);

    private final Scheduler scheduler = new Scheduler(this, new SubSystem[] {limelight, collection, delivery, driveBase, odometry, hang});

    List<LynxModule> allHubs;

    ElapsedTime autoTime = new ElapsedTime();

    ElapsedTime timer = new ElapsedTime();
    double lastTime;
    public double loopTime;

    public Gamepad currentGamepad1 = new Gamepad();
    public Gamepad lastGamepad1 = new Gamepad();

    public Gamepad currentGamepad2 = new Gamepad();
    public Gamepad lastGamepad2 = new Gamepad();

    public RobotPower RobotPosition = new RobotPower();

    public abstract void initEX();

    public abstract void loopEX();

    @Override
    public void init() {

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        timer.reset();
        scheduler.init();
        initEX();
    }

    @Override
    public void start() {
        autoTime.reset();
        limelight.onStart();
        super.start();
    }

    @Override
    public void loop() {
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        lastGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);

        lastGamepad2.copy(currentGamepad2);
        currentGamepad2.copy(gamepad2);

        lastTime = timer.milliseconds();

        RobotPosition = new RobotPower(odometry.X(), odometry.Y(), odometry.Heading());
        collection.updateRobotPosition(RobotPosition);
        limelight.updatePythonInputs(odometry.X(), odometry.Y(), odometry.Heading(), delivery.getSlidePositionCM());
        collection.updateDelivery(delivery);

        scheduler.execute();
        loopEX();

        if (delivery.transferFailed){
            collection.clearQueue();

            collection.queueCommand(collection.retryTransfer);

            collection.queueCommand(delivery.closeGripper);

            collection.queueCommand(collection.openGripper);

            delivery.transferFailed = false;
        }

//        System.out.println("collection slides: " + collection.horizontalMotor.getCurrentDraw());
//        System.out.println("delivery slide 1: " + delivery.slideMotor.getCurrentDraw());
//        System.out.println("delivery slide 2: " + delivery.slideMotor2.getCurrentDraw());
//        System.out.println("hang motor: " + hang.hangPower.getCurrentDraw());
//        System.out.println("Right front: " + driveBase.RF.getCurrentDraw());
//        System.out.println("Left front: " + driveBase.LF.getCurrentDraw());
//        System.out.println("Right back: " + driveBase.RB.getCurrentDraw());
//        System.out.println("Left back: " + driveBase.LB.getCurrentDraw());

        loopTime = timer.milliseconds() - lastTime;
        System.out.println("loop time: " +loopTime);
    }

    /**
     * Use this to write data to the log
     * */
    @Override
    public void stop() {
//        collection.disableServos();
//        delivery.disableServos();
//        super.stop();
    }
}
