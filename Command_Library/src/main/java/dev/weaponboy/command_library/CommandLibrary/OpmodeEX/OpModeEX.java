package dev.weaponboy.command_library.CommandLibrary.OpmodeEX;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import dev.weaponboy.command_library.CommandLibrary.Subsystem.SubSystem;
import dev.weaponboy.command_library.Subsystems.DriveBase;
import dev.weaponboy.command_library.Subsystems.Odometry;
import dev.weaponboy.command_library.Subsystems.horizontalSlides;

public abstract class OpModeEX extends OpMode {

    public DriveBase driveBase = new DriveBase(this);

    public horizontalSlides horizontalslides = new horizontalSlides(this);

    public Odometry odometry = new Odometry(this);

    private final Scheduler scheduler = new Scheduler(this, new SubSystem[] {driveBase, horizontalslides, odometry});

    public abstract void initEX();

    public abstract void loopEX();

    @Override
    public void init() {
        scheduler.init();
        initEX();
    }

    @Override
    public void loop() {
        scheduler.execute();
        loopEX();
    }

    /**
     * Use this to write data to the log
     * */
    @Override
    public void stop() {
//        super.stop();
    }
}
