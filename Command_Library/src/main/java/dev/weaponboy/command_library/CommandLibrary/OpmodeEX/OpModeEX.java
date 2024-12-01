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
import dev.weaponboy.command_library.Subsystems.Odometry;
import dev.weaponboy.nexus_pathing.PathingUtility.RobotPower;

public abstract class OpModeEX extends OpMode {

    public DriveBase driveBase = new DriveBase(this);

    public Collection collection = new Collection(this);

    public Delivery delivery = new Delivery(this);

    public Odometry odometry = new Odometry(this);

    public Hang hang = new Hang(this);
//collection, delivery, driveBase, odometry
    private final Scheduler scheduler = new Scheduler(this, new SubSystem[] {collection, delivery, driveBase, odometry});

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

        scheduler.execute();
        loopEX();

        loopTime = timer.milliseconds() - lastTime;
//        System.out.println("loop time: " +loopTime);
    }

    /**
     * Use this to write data to the log
     * */
    @Override
    public void stop() {
        collection.disableServos();
//        delivery.disableServos();
//        super.stop();
    }
}
