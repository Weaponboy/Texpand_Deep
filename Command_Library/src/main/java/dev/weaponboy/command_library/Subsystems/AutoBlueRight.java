package dev.weaponboy.command_library.Subsystems;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;
import dev.weaponboy.nexus_pathing.Follower.follower;
import dev.weaponboy.nexus_pathing.PathGeneration.commands.sectionBuilder;
import dev.weaponboy.nexus_pathing.PathGeneration.pathsManager;
import dev.weaponboy.nexus_pathing.PathingUtility.RobotPower;
import dev.weaponboy.nexus_pathing.RobotUtilities.Vector2D;

@Autonomous(name = "blueRight", group = "Autos")
public class AutoBlueRight extends OpModeEX {


    pathsManager paths = new pathsManager();
    follower follow = new follower();


    private final sectionBuilder[] rightBluePath = {
            () -> paths.addPoints(new Vector2D(147, 360), new Vector2D(151, 245), new Vector2D(50.4, 329))

    };

    private final sectionBuilder[] rightBlueFull = {
            () -> paths.addPoints(new Vector2D(151, 340), new Vector2D(160, 244), new Vector2D(323, 325)),
            () -> paths.addPoints(new Vector2D(55, 330), new Vector2D(160, 240))
    };
    private final sectionBuilder[] rightBlueBasket = {
            () -> paths.addPoints(new Vector2D(147, 360), new Vector2D(160, 244))
    };

    public void runOpMode() throws InterruptedException {
        //public  enum autoState {
          //  preload,
//            collectingSpike,
//            delivering,
//            collectingSub,
//        }
//         autoState state = rightBlueBasket.autoState.preload;

        }





    @Override
    public void initEX() {
        paths.addNewPath("rightBluePath");
        paths.buildPath(rightBluePath);
        follow.setPath(paths.returnPath("rightBluePath"));


        paths.addNewPath("rightBlueBasket");
        paths.buildPath(rightBlueBasket);
        follow.setPath(paths.returnPath("rightBlueBasket"));


        paths.addNewPath("rightBlueFull");
        paths.buildPath(rightBlueFull);
        follow.setPath(paths.returnPath("rightBlueFull"));
    }

    @Override
    public void loopEX() {
        RobotPower currentPower = follow.followPathAuto(180, odometry.Heading(), odometry.X(), odometry.Y(), 180, 180);
        driveBase.queueCommand(driveBase.drivePowers(currentPower.getVertical(), currentPower.getHorizontal(), currentPower.getPivot()));
    }
}
