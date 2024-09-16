package org.firstinspires.ftc.teamcode.Examples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import dev.weaponboy.nexus_pathing.Follower.follower;
import dev.weaponboy.nexus_pathing.PathGeneration.commands.sectionBuilder;
import dev.weaponboy.nexus_pathing.PathGeneration.pathsManager;
import dev.weaponboy.nexus_pathing.RobotUtilities.Vector2D;

@Autonomous(name = "testing_pathing", group = "Test Autos")
public class Pathing_Example extends LinearOpMode {

    pathsManager paths = new pathsManager();

    follower follower = new follower();

    private final sectionBuilder[] blueLeft = {
            () -> paths.addPoints(new Vector2D(0,0), new Vector2D(45, 60)),
            () -> paths.addPoints(new Vector2D(45, 60), new Vector2D(100, 60), new Vector2D(100, 100))
    };

    @Override
    public void runOpMode() throws InterruptedException {

        paths.addNewPath("blue_left");

        paths.buildPath(blueLeft);

    }

}
