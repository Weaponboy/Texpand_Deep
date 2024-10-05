package org.firstinspires.ftc.teamcode.Examples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;
import dev.weaponboy.nexus_pathing.Follower.follower;
import dev.weaponboy.nexus_pathing.PathGeneration.commands.sectionBuilder;
import dev.weaponboy.nexus_pathing.PathGeneration.pathsManager;
import dev.weaponboy.nexus_pathing.RobotUtilities.Vector2D;

@Autonomous(name = "pathing example", group = "Test Autos")
public class Pathing_Example extends OpModeEX {

    pathsManager paths = new pathsManager();

    follower follower = new follower();

    double targetHeading;

    private final sectionBuilder[] line = {
            () -> paths.addPoints(new Vector2D(0, 0), new Vector2D(50, 0), new Vector2D(100, 0))
    };

    public enum autoState{
        preload,
        collectingSpike,
        delivering,
        collectingSub
    }

    public enum building{
        built,
        notBuilt
    }

    public autoState state = Pathing_Example.autoState.preload;
    public building built = building.notBuilt;

    @Override
    public void initEX() {
        paths.addNewPath("line");
        paths.buildPath(line);
    }

    @Override
    public void loopEX() {

        if (state == autoState.preload){

            if (built == building.notBuilt){
                follower.setPath(paths.returnPath("line"));
                targetHeading = 0;
                built = building.built;
            }

//            if (pathFinished){
//                state = autoState.collectingSpike;
//                built = building.notBuilt;
//            }

        } else if (state == autoState.collectingSpike) {

            if (built == building.notBuilt){
                follower.setPath(paths.returnPath("line"));
                targetHeading = 0;
                built = building.built;
            }

        }

        telemetry.addData("Paths.size", paths.paths.size());

        //update methods
        odometry.queueCommand(odometry.update);
        driveBase.queueCommand(driveBase.drivePowers(follower.followPathAuto(targetHeading, odometry.Heading(), odometry.X(), odometry.Y(), 180, 180)));

    }




}
