package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Examples.Pathing_Example;

import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;
import dev.weaponboy.command_library.Subsystems.Collection;
import dev.weaponboy.command_library.Subsystems.Delivery;
import dev.weaponboy.nexus_pathing.Follower.follower;
import dev.weaponboy.nexus_pathing.PathGeneration.commands.sectionBuilder;
import dev.weaponboy.nexus_pathing.PathGeneration.pathsManager;
import dev.weaponboy.nexus_pathing.PathingUtility.RobotPower;
import dev.weaponboy.nexus_pathing.RobotUtilities.Vector2D;
import dev.weaponboy.nexus_pathing.PathGeneration.pathBuilder;
@Autonomous(name = "testing_pathing", group = "Test Autos")
public class Blue_Left extends OpModeEX {

    pathsManager paths = new pathsManager();
    follower follow = new follower();
    double targetHeading;
    double cycles=0;
    boolean drop;
    ElapsedTime dropTimer=new ElapsedTime();


    public enum autoState{
        preload,
        collectingSpike,
        delivering,
        collectingSub,
        finished,
    }

    public enum building{
        built,
        notBuilt
    }
    public enum targetAuto{
        preload,
        spikes,
        sub,


    }


    public autoState state = autoState.preload;
    public building built = building.notBuilt;
    public targetAuto size = targetAuto.preload;

    private final sectionBuilder[] preloadPath = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(339, 202), new Vector2D(290.3, 240.5), new Vector2D(325.3, 324)),

    };

    private final sectionBuilder[] spike1Pickup = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(325.3, 324), new Vector2D(300, 315.4), new Vector2D(267, 303.6)),

    };
    private final sectionBuilder[] spike1Drop = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(267, 303.6), new Vector2D(300, 315.4), new Vector2D(325.3, 324)),

    };
    private final sectionBuilder[] spike2Pickup = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(325.3, 324), new Vector2D(311.7, 333.7), new Vector2D(267, 330.2)),

    };
    private final sectionBuilder[] spike2Drop = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(267, 330.2), new Vector2D(311.7, 333.7), new Vector2D(325.3, 324)),

    };
    private final sectionBuilder[] spike3Pickup = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(325.3, 324), new Vector2D(256.4, 303), new Vector2D(246.7, 331.2)),

    };
    private final sectionBuilder[] spike3Drop = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(246.7, 331.2), new Vector2D(256.4, 303), new Vector2D(325.3, 324)),

    };
    @Override
    public void initEX() {

        odometry.startPosition(339, 202, 270);

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
            //preLoad
        if (state == autoState.preload){

            if (built == building.notBuilt){
                delivery.queueCommand(delivery.transfer);
                delivery.queueCommand(delivery.slideSetPonts(delivery.highBasket));
                delivery.queueCommand(delivery.deposit);
                follow.setPath(paths.returnPath("preloadPath"));
                targetHeading = 225;
                built = building.built;
                drop=true;
                dropTimer.reset();
            }
            //spike1

            if (delivery.fourbarState== Delivery.fourBarState.basketDeposit&&drop&&dropTimer.milliseconds()>5000){
                delivery.queueCommand(delivery.deposit);
                delivery.queueCommand(delivery.deposit);
                delivery.queueCommand(delivery.slideSetPonts(0));
                drop=false;

            }

            if (follow.isFinished()&&delivery.fourbarState== Delivery.fourBarState.behindNest){


                if (size==targetAuto.preload){

                     state=autoState.finished;

                 }else if (size!=targetAuto.preload){
                     state = autoState.collectingSpike;
                     built = building.notBuilt;
                 }

           }


        } else if (state == autoState.collectingSpike ) {

            if (built == building.notBuilt){
                follow.setPath(paths.returnPath("spike1Pickup"));
                targetHeading = 0;
                built = building.built;
            }
            if (follow.isFinished()){
                state = autoState.delivering;
                built = building.notBuilt;
            }
        } else if ( state == autoState.delivering) {

            if (built == building.notBuilt){
                follow.setPath(paths.returnPath("spike1Drop"));
                targetHeading = 0;
                built = building.built;
            }
            if (follow.isFinished()&&cycles<0){
                state = autoState.collectingSpike;
                built = building.notBuilt;
            }
            //spike 2

        }
//
//        {
//
//            if (built == building.notBuilt){
//                follow.setPath(paths.returnPath("spike2Pickup"));
//                targetHeading = 0;
//                built = building.built;
//            }
//            if (follow.isFinished()){
//                state = autoState.delivering;
//                built = building.notBuilt;
//            }
//        }  {
//
//            if (built == building.notBuilt){
//                follow.setPath(paths.returnPath("spike2Drop"));
//                targetHeading = 0;
//                built = building.built;
//            }
//
//            //spike 3
//
//        }  {
//
//            if (built == building.notBuilt){
//                follow.setPath(paths.returnPath("spike3Pickup"));
//                targetHeading = 0;
//                built = building.built;
//            }
//            if (follow.isFinished()){
//                state = autoState.delivering;
//                built = building.notBuilt;
//            }
//        }  {
//
//            if (built == building.notBuilt){
//                follow.setPath(paths.returnPath("spike3Drop"));
//                targetHeading = 0;
//                built = building.built;
//            }
//
//        }

        if (state==autoState.finished){
            requestOpModeStop();
        }

        odometry.queueCommand(odometry.updateLineBased);
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
