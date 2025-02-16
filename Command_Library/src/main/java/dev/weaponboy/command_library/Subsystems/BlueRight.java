package dev.weaponboy.command_library.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;
import dev.weaponboy.nexus_pathing.Follower.follower;
import dev.weaponboy.nexus_pathing.PathGeneration.commands.sectionBuilder;
import dev.weaponboy.nexus_pathing.PathGeneration.pathsManager;
import dev.weaponboy.nexus_pathing.PathingUtility.RobotPower;
import dev.weaponboy.nexus_pathing.RobotUtilities.Vector2D;
@Autonomous (name = "blueRight", group = "Autos")
public class BlueRight extends OpModeEX {
    double targetHeading;
    pathsManager paths = new pathsManager();
    follower follow = new follower();

    private final sectionBuilder[] right1x1 = {
            () -> paths.addPoints((new Vector2D(147, 360)), new Vector2D(151, 245))

    };
    private final sectionBuilder[] rightObv = {
            () -> paths.addPoints((new Vector2D(151,245)), new Vector2D(58,331))

    };


    public enum autoState {
        preload,
        clip,
        preLoad,
        finished,
        deposit,


    }

    public enum building {
        built,
        notBuilt, notbuilt
    }

    public enum autoTarget {

        preLoad,


    }


   autoState state = autoState.preLoad;
     public building built = building.built;
autoTarget target = autoTarget.preLoad;
    @Override
    public void initEX() {
        paths.addNewPath("right1x1");
        paths.buildPath(right1x1);
        follow.setPath(paths.returnPath("right1x1"));

        paths.addNewPath("rightObv");
        paths.buildPath(right1x1);
        follow.setPath(paths.returnPath("rightObv"));


    }

    @Override
    public void loopEX() {

        if (state == autoState.preLoad) {


            if (built== building.notbuilt) {

                follow.setPath(paths.returnPath("rightBluePath"));
                targetHeading = 0;
                built = building.built;
                delivery.queueCommand(delivery.Clip);
            }
            if (follow.isFinished ()) {
                if (target == autoTarget.preLoad) {
                    state = autoState.finished;
                }
            } else  {
                state = autoState.clip;
                built = building.notBuilt;
            }
        } else if (state == autoState.deposit) {

            if (built == building.notBuilt) {
                follow.setPath(paths.returnPath("rightObv"));
                targetHeading = 0;
                built = building.built;
                collection.queueCommand(collection.collect);

            }
                RobotPower currentPower = follow.followPathAuto(180, odometry.Heading(), odometry.X(), odometry.Y(), 180, 180);
                driveBase.queueCommand(driveBase.drivePowers(currentPower.getVertical(), currentPower.getHorizontal(), currentPower.getPivot()));


            }
        }
    }



