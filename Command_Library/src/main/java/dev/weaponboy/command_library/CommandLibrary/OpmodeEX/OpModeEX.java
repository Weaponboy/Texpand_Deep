package dev.weaponboy.command_library.CommandLibrary.OpmodeEX;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

import dev.weaponboy.command_library.CommandLibrary.Subsystem.SubSystem;
import dev.weaponboy.command_library.Subsystems.Delivery;
import dev.weaponboy.command_library.Subsystems.DriveBase;
import dev.weaponboy.command_library.Subsystems.Odometry;
import dev.weaponboy.command_library.Subsystems.Collection;

public abstract class OpModeEX extends OpMode {

    public DriveBase driveBase = new DriveBase(this);

    public Collection collection = new Collection(this);

    public Delivery delivery = new Delivery(this);

    public Odometry odometry = new Odometry(this);

    private final Scheduler scheduler = new Scheduler(this, new SubSystem[] {driveBase, odometry, collection, delivery});

    List<LynxModule> allHubs;

    ElapsedTime timer = new ElapsedTime();
    double lastTime;
    public double loopTime;

    public Gamepad currentGamepad1 = new Gamepad();
    public Gamepad lastGamepad1 = new Gamepad();

    public Gamepad currentGamepad2 = new Gamepad();
    public Gamepad lastGamepad2 = new Gamepad();

    public abstract void initEX();

    public abstract void loopEX();

    @Override
    public void init() {
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        timer.reset();
        initEX();
        scheduler.init();

    }

    @Override
    public void loop() {
        lastGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);

        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        lastTime = timer.milliseconds();

        try {
            scheduler.execute();
            loopEX();
        } catch (Exception e) {

            collection.safePositions();
            delivery.safePositions();

            collection.disableServos();
            delivery.disableServos();
        } finally {
            collection.disableServos();
            delivery.disableServos();
        }

        loopTime = timer.milliseconds() - lastTime;
    }

    /**
     * Use this to write data to the log
     * */
    @Override
    public void stop() {
        collection.disableServos();
        delivery.disableServos();
//        super.stop();
    }
}
