package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Examples.Pathing_Example;

import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;
import dev.weaponboy.command_library.Subsystems.Collection;
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
    }


    public autoState state = autoState.preload;
    public building built = building.notBuilt;
    public targetAuto size = targetAuto.preload;

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
            //preLoad
        if (state == autoState.preload){

            if (built == building.notBuilt){
                follow.setPath(paths.returnPath("preloadPath"));
                targetHeading = 270;
                built = building.built;
            }
            //spike1

            if (follow.isFinished()){
                 if (size==targetAuto.preload){
                     state=autoState.finished;

                 }else {
                     state = autoState.collectingSpike;
                     built = building.notBuilt;}

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

        }  {

            if (built == building.notBuilt){
                follow.setPath(paths.returnPath("spike2Pickup"));
                targetHeading = 0;
                built = building.built;
            }
            if (follow.isFinished()){
                state = autoState.delivering;
                built = building.notBuilt;
            }
        }  {

            if (built == building.notBuilt){
                follow.setPath(paths.returnPath("spike2Drop"));
                targetHeading = 0;
                built = building.built;
            }

            //spike 3

        }  {

            if (built == building.notBuilt){
                follow.setPath(paths.returnPath("spike3Pickup"));
                targetHeading = 0;
                built = building.built;
            }
            if (follow.isFinished()){
                state = autoState.delivering;
                built = building.notBuilt;
            }
        }  {

            if (built == building.notBuilt){
                follow.setPath(paths.returnPath("spike3Drop"));
                targetHeading = 0;
                built = building.built;
            }

        }
        if (state==autoState.finished){
            requestOpModeStop();
        }

        RobotPower currentPower = follow.followPathAuto(180, odometry.Heading(), odometry.X(), odometry.Y(), 180, 180);
        driveBase.queueCommand(driveBase.drivePowers(currentPower.getVertical(), currentPower.getHorizontal(), currentPower.getPivot()));

    }

}
