package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;
import dev.weaponboy.nexus_pathing.Follower.follower;
import dev.weaponboy.nexus_pathing.PathGeneration.commands.sectionBuilder;
import dev.weaponboy.nexus_pathing.PathGeneration.pathsManager;
import dev.weaponboy.nexus_pathing.PathingUtility.RobotPower;
import dev.weaponboy.nexus_pathing.RobotUtilities.Vector2D;

@Autonomous(name = "testing_pathing", group = "Test Autos")
public class ExampleAuto extends OpModeEX {

    pathsManager paths = new pathsManager();
    follower follow = new follower();

    private final sectionBuilder[] preloadPath = {
            () -> paths.addPoints(new Vector2D(341, 210), new Vector2D(331, 268), new Vector2D(336, 328))
    };

    @Override
    public void initEX() {
        paths.addNewPath("preloadPath");

        paths.buildPath(preloadPath);

        follow.setPath(paths.returnPath("preloadPath"));
    }

    @Override
    public void loopEX() {

        RobotPower currentPower = follow.followPathAuto(180, odometry.Heading(), odometry.X(), odometry.Y(), 180, 180);

        driveBase.queueCommand(driveBase.drivePowers(currentPower.getVertical(), currentPower.getHorizontal(), currentPower.getPivot()));

    }

}
