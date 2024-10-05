package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.Examples.Pathing_Example;

import dev.weaponboy.nexus_pathing.Follower.follower;
import dev.weaponboy.nexus_pathing.PathGeneration.commands.sectionBuilder;
import dev.weaponboy.nexus_pathing.PathGeneration.pathsManager;
import dev.weaponboy.nexus_pathing.RobotUtilities.RobotConfig;
import dev.weaponboy.nexus_pathing.RobotUtilities.Vector2D;

public class TestingPath {

    static pathsManager paths = new pathsManager();

    static follower follower = new follower();

    double targetHeading;

    static final sectionBuilder[] line = {
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

    public Pathing_Example.autoState state = Pathing_Example.autoState.preload;
    public Pathing_Example.building built = Pathing_Example.building.notBuilt;

    public static void main(String[] args) {
        paths.addNewPath("line");
        paths.buildPath(line);

//        System.out.println(config.MAX_X_VELOCITY());
    }

}
