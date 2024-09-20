package dev.weaponboy.command_library.CommandLibrary.OpmodeEX;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import dev.weaponboy.command_library.CommandLibrary.Subsystem.SubSystem;
import dev.weaponboy.command_library.Subsystems.DriveBase;
import dev.weaponboy.command_library.Subsystems.Odometry;

public abstract class OpModeEX extends OpMode {

    public DriveBase driveBase = new DriveBase(this);
    public horizontalSlides horizontalSlides = new horizontalSlides(this);

    public Odometry odometry = new Odometry(this);

    private Scheduler scheduler = new Scheduler(this, new SubSystem[] {driveBase});


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
