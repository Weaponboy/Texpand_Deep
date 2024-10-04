package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;
import dev.weaponboy.command_library.Subsystems.Collection;
import dev.weaponboy.nexus_pathing.Follower.follower;
import dev.weaponboy.nexus_pathing.PathGeneration.commands.sectionBuilder;
import dev.weaponboy.nexus_pathing.PathGeneration.pathsManager;
import dev.weaponboy.nexus_pathing.PathingUtility.RobotPower;
import dev.weaponboy.nexus_pathing.RobotUtilities.Vector2D;

@Autonomous(name = "testing_pathing", group = "Test Autos")
public class Blue_Left extends OpModeEX {

    pathsManager paths = new pathsManager();
    follower follow = new follower();
    enum autoState{
        preload,
        spike1,
        spike1Retern,
        spike2,
        spike2Retern,
        spike3,
        spike3Retern,
    }
    enum building{
        notBuilt,
        built,
    }
    //private Blue_Left.autoState autoState = Blue_Left.autoState.pre;

    private final sectionBuilder[] preloadPath = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(271, 335), new Vector2D(308.5, 332.3), new Vector2D(335.7, 335.3)),

    };
    private final sectionBuilder[] spike1Pickup = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(335.7, 335.3), new Vector2D(315.4, 300), new Vector2D(303.6, 267)),

    };
    private final sectionBuilder[] spike1Drop = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(303.6, 267), new Vector2D(315.4, 300), new Vector2D(335.7, 335.3)),

    };
    private final sectionBuilder[] spike2Pickup = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(335.7, 335.3), new Vector2D(333.7, 311.7), new Vector2D(330.2, 267)),

    };
    private final sectionBuilder[] spike2Drop = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(330.2, 267), new Vector2D(333.7, 311.7), new Vector2D(335.7, 335.3)),

    };
    private final sectionBuilder[] spike3Pickup = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(335.7, 335.3), new Vector2D(303, 256.4), new Vector2D(331.2, 246.7)),

    };
    private final sectionBuilder[] spike3Drop = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(331.2, 246.7), new Vector2D(303, 256.4), new Vector2D(335.7, 335.3)),

    };
    @Override
    public void initEX() {
        paths.addNewPath("preloadPath");

        paths.buildPath(preloadPath);

        follow.setPath(paths.returnPath("preloadPath"));




        //spike1 paths
        paths.addNewPath("spike1Pickup");

        paths.buildPath(spike1Pickup);

        follow.setPath(paths.returnPath("spike1Pickup"));



        paths.addNewPath("spike1Drop");

        paths.buildPath(spike1Drop);

        follow.setPath(paths.returnPath("spike1Drop"));



        //spike2 paths
        paths.addNewPath("spike2Pickup");

        paths.buildPath(spike2Pickup);

        follow.setPath(paths.returnPath("spike2Pickup"));

        paths.addNewPath("spike2Drop");

        paths.buildPath(spike2Drop);

        follow.setPath(paths.returnPath("spike2Drop"));





        //spike3 paths
        paths.addNewPath("spike3Pickup");

        paths.buildPath(spike3Pickup);

        follow.setPath(paths.returnPath("spike3Pickup"));

        paths.addNewPath("spike3Drop");

        paths.buildPath(spike3Drop);

        follow.setPath(paths.returnPath("spike3Drop"));

    }

    @Override
    public void loopEX() {

        RobotPower currentPower = follow.followPathAuto(180, odometry.Heading(), odometry.X(), odometry.Y(), 180, 180);

        driveBase.queueCommand(driveBase.drivePowers(currentPower.getVertical(), currentPower.getHorizontal(), currentPower.getPivot()));

    }

}
