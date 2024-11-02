package org.firstinspires.ftc.teamcode.Auto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;
import dev.weaponboy.command_library.Subsystems.Collection;
import dev.weaponboy.command_library.Subsystems.Delivery;
import dev.weaponboy.nexus_pathing.Follower.follower;
import dev.weaponboy.nexus_pathing.PathGeneration.commands.sectionBuilder;
import dev.weaponboy.nexus_pathing.PathGeneration.pathsManager;
import dev.weaponboy.nexus_pathing.PathingUtility.RobotPower;
import dev.weaponboy.nexus_pathing.RobotUtilities.Vector2D;

@Autonomous(name = "Blue Right", group = "Autos")
public class Blue_Right extends OpModeEX {
    double targetHeading;


    pathsManager paths = new pathsManager();
    follower follow = new follower();


    private final sectionBuilder[] rightBluePath = {
            () -> paths.addPoints(new Vector2D(339, 160), new Vector2D(255, 170))

    };
    private final sectionBuilder[] rightBlueHumanPlayer = {
            () -> paths.addPoints(new Vector2D(255, 170), new Vector2D(319, 211),new Vector2D(333, 83))
    };

    private final sectionBuilder[] rightBlueFull = {
            () -> paths.addPoints(new Vector2D(151, 340), new Vector2D(160, 244), new Vector2D(323, 325)),
            () -> paths.addPoints(new Vector2D(55, 330), new Vector2D(160, 240))
    };
    private final sectionBuilder[] rightBlueBasket = {
            () -> paths.addPoints(new Vector2D(147, 360), new Vector2D(160, 244))
    };

    public enum autoState {
        preLoad,
        deposit,
        collect,
        humanPlayer,
        finished
    }

    public enum building {
        built,
        notBuilt
    }

    boolean following = false;
    boolean pulleldslidesin = false;

    autoState state = autoState.preLoad;
    building built = building.notBuilt;

    @Override
    public void initEX() {
        collection.pullOut.execute();

        odometry.startPosition(339, 160, 180);


        paths.addNewPath("rightBluePath");
        paths.buildPath(rightBluePath);


        paths.addNewPath("rightBlueBasket");
        paths.buildPath(rightBlueBasket);

        paths.addNewPath("rightBlueHumanPlayer");
        paths.buildPath(rightBlueHumanPlayer);


        paths.addNewPath("rightBlueFull");
        paths.buildPath(rightBlueFull);
        follow.setPath(paths.returnPath("rightBluePath"));

        delivery.queueCommand(delivery.transfer);
        delivery.queueCommand(delivery.slideSetPonts(delivery.highChamber));
    }


    @Override
    public void loopEX() {

        if (state == autoState.preLoad) {

            if (built == building.notBuilt && delivery.slideMotor.getCurrentPosition() > 350) {
                follow.setPath(paths.returnPath("rightBluePath"));
                targetHeading = 180;
                following = true;
                built = building.built;
                delivery.queueCommand(delivery.transfer);
                delivery.queueCommand(delivery.Clip);

            }

            if (follow.isFinished() && !pulleldslidesin) {
                pulleldslidesin = true;
                try {
                    Thread.sleep(500);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                delivery.queueCommand(delivery.Clip);
                delivery.queueCommand(delivery.Clip);
                delivery.queueCommand(delivery.slideSetPonts(0));
            } else if (follow.isFinished() && delivery.slideMotor.getCurrentPosition() < 20) {
                state = autoState.humanPlayer;
                built = building.notBuilt;
                try {
                    Thread.sleep(500);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
            }

        } else if (state == autoState.humanPlayer) {

            if (state == autoState.humanPlayer) {

                if (built == building.notBuilt) {
                    follow.setPath(paths.returnPath("rightBlueHumanPlayer"));
                    targetHeading = 315;
                    following = true;
                    built = building.built;
               
                }
                
                if (follow.isFinished() && collection.getFourBarState() == Collection.fourBar.transferUp) {
                    collection.queueCommand(collection.collect);
                    collection.queueCommand(collection.collect);
                    

//

//
//

                } else if (follow.isFinished() && !collection.nestSensor.isPressed()) {

                    state = autoState.finished;
                    
                }

            }

        }

        if (state == autoState.finished) {
            requestOpModeStop();
        }



        odometry.queueCommand(odometry.updateLineBased);

        if (following) {
            RobotPower currentPower = follow.followPathAuto(targetHeading, odometry.Heading(), odometry.X(), odometry.Y(), odometry.getXVelocity(), odometry.getYVelocity());
            telemetry.addData("Loop time", loopTime);
            telemetry.addData("Y", odometry.Y());
            telemetry.addData("Heading", odometry.Heading());
            telemetry.addData("X", odometry.X());
            telemetry.addData("getVertical", currentPower.getVertical());
            telemetry.addData("getHorizontal", currentPower.getHorizontal());
            telemetry.addData("getPivot", currentPower.getPivot());
            telemetry.update();

            driveBase.queueCommand(driveBase.drivePowers(currentPower));
        }
    }
}
