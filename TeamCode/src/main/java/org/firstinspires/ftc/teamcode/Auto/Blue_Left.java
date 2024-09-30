package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;
import dev.weaponboy.nexus_pathing.Follower.follower;
import dev.weaponboy.nexus_pathing.PathGeneration.commands.sectionBuilder;
import dev.weaponboy.nexus_pathing.PathGeneration.pathsManager;
import dev.weaponboy.nexus_pathing.PathingUtility.RobotPower;
import dev.weaponboy.nexus_pathing.RobotUtilities.Vector2D;

@Autonomous(name = "testing_pathing", group = "Test Autos")
public class Blue_Left extends OpModeEX {

    pathsManager paths = new pathsManager();
    follower follow = new follower();

    private final sectionBuilder[] preloadPath = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(271, 335), new Vector2D(308.5, 332.3), new Vector2D(335.7, 335.3)),
            () -> paths.addPoints(new Vector2D(335.7, 335.3), new Vector2D(315.4, 300), new Vector2D(303.6, 267)),
            () -> paths.addPoints(new Vector2D(303.6, 267), new Vector2D(315.4, 300), new Vector2D(335.7, 335.3)),
            () -> paths.addPoints(new Vector2D(335.7, 335.3), new Vector2D(333.7, 311.7), new Vector2D(330.2, 267)),
            () -> paths.addPoints(new Vector2D(330.2, 267), new Vector2D(333.7, 311.7), new Vector2D(335.7, 335.3)),
            () -> paths.addPoints(new Vector2D(335.7, 335.3), new Vector2D(303, 256.4), new Vector2D(331.2, 246.7)),
            () -> paths.addPoints(new Vector2D(331.2, 246.7), new Vector2D(303, 256.4), new Vector2D(335.7, 335.3)),

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
