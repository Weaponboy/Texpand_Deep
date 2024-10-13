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
@Autonomous(name = "Blue Left", group = "Test Autos")
public class Blue_Left extends OpModeEX {

    pathsManager paths = new pathsManager();
    follower follow = new follower();
    double targetHeading;
    double cycles=0;
    boolean drop;
    ElapsedTime dropTimer=new ElapsedTime();
    boolean collect;
    ElapsedTime collectTimer=new ElapsedTime();


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
            () -> paths.addPoints(new Vector2D(339, 202), new Vector2D(290.3, 240.5), new Vector2D(329, 321)),

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
        telemetry.addData("type",size);
        telemetry.update();

        if (!lastGamepad1.b&& currentGamepad1.b){
             size = targetAuto.spikes;
             cycles=1;
            telemetry.addData("type",size);
            telemetry.addData("cycles",cycles);
             telemetry.update();
        }
        if (!lastGamepad1.a&& currentGamepad1.a){
            size = targetAuto.spikes;
            cycles=2;
            telemetry.addData("type",size);
            telemetry.addData("cycles",cycles);
            telemetry.update();
        }
//        if (!lastGamepad1.x&& currentGamepad1.x){
//            size = targetAuto.spikes;
//            cycles=3;
//            telemetry.addData("type",size);
//            telemetry.addData("cycles",cycles);
//            telemetry.update();
//
//        }

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
        if (state == autoState.preload||state == autoState.collectingSpike||state == autoState.collectingSub){

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


            if (delivery.fourbarState== Delivery.fourBarState.basketDeposit&&drop&&dropTimer.milliseconds()>4000){

                delivery.queueCommand(delivery.deposit);

                delivery.queueCommand(delivery.deposit);

                delivery.queueCommand(delivery.slideSetPonts(0));

                drop=false;

            }

            if (follow.isFinished()&&delivery.fourbarState== Delivery.fourBarState.behindNest && delivery.slideMotor.getCurrentPosition() < 20){


                if (size==targetAuto.preload){

                     state=autoState.finished;

                }else if (size!=targetAuto.preload){
                     state = autoState.collectingSpike;
                     built = building.notBuilt;
                }

           }

            //spike1
        } else if (state == autoState.collectingSpike||state == autoState.collectingSub&&cycles>0.9 ) {

            if (built == building.notBuilt){
                follow.setPath(paths.returnPath("spike1Pickup"));
                targetHeading = 180;
                built = building.built;
                collection.queueCommand(collection.collect);
            }
            if (follow.isFinished()){
                collect=true;
                collectTimer.reset();
                collection.griperRotate.setPosition(45);
                collection.queueCommand(collection.collect);
                if (collect&&collectTimer.milliseconds()>4000){
                    collection.queueCommand(collection.transfer);
                    collect=false;
                }


                 if (size!=targetAuto.spikes&&cycles>1&&!collect){
                    state = autoState.collectingSpike;
                    built = building.notBuilt;
                }

            }
        } else if ( state == autoState.delivering) {

            if (built == building.notBuilt){
                delivery.queueCommand(delivery.transfer);
                delivery.queueCommand(delivery.slideSetPonts(delivery.highBasket));
                delivery.queueCommand(delivery.deposit);
                follow.setPath(paths.returnPath("spike1Drop"));
                targetHeading = 225;
                drop=true;
                dropTimer.reset();
                built = building.built;

            }
            if (follow.isFinished()){

                if (delivery.fourbarState== Delivery.fourBarState.basketDeposit&&drop&&dropTimer.milliseconds()>4000){

                    delivery.queueCommand(delivery.deposit);

                    delivery.queueCommand(delivery.deposit);

                    delivery.queueCommand(delivery.slideSetPonts(0));

                    drop=false;

                }


                if (size==targetAuto.spikes&&cycles<1.1&&collection.horizontalMotor.getCurrentPosition()<10){

                    state=autoState.finished;

                }else if (size!=targetAuto.spikes&&cycles>1&&collection.horizontalMotor.getCurrentPosition()<10){
                    state = autoState.collectingSpike;
                    built = building.notBuilt;
                }

            }
            //spike 2
        } else if (state == autoState.collectingSpike||state == autoState.collectingSub&&cycles>1.9 ) {

            if (built == building.notBuilt){
                follow.setPath(paths.returnPath("spike1Pickup"));
                targetHeading = 180;
                built = building.built;
                collection.queueCommand(collection.collect);
            }
            if (follow.isFinished()){
                collect=true;
                collectTimer.reset();
                collection.griperRotate.setPosition(45);
                collection.queueCommand(collection.collect);
                if (collect&&collectTimer.milliseconds()>700){
                    collection.queueCommand(collection.transfer);
                    collect=false;
                }


                if (size!=targetAuto.spikes&&cycles>2&&!collect){
                    state = autoState.collectingSpike;
                    built = building.notBuilt;
                }

            }
        } else if ( state == autoState.delivering) {

            if (built == building.notBuilt){
                delivery.queueCommand(delivery.transfer);
                delivery.queueCommand(delivery.slideSetPonts(delivery.highBasket));
                delivery.queueCommand(delivery.deposit);
                follow.setPath(paths.returnPath("spike1Drop"));
                targetHeading = 225;
                drop=true;
                dropTimer.reset();
                built = building.built;

            }
            if (follow.isFinished()){

                if (delivery.fourbarState== Delivery.fourBarState.basketDeposit&&drop&&dropTimer.milliseconds()>4000){

                    delivery.queueCommand(delivery.deposit);

                    delivery.queueCommand(delivery.deposit);

                    delivery.queueCommand(delivery.slideSetPonts(0));

                    drop=false;

                }


                if (size==targetAuto.spikes&&cycles<2.1&&collection.horizontalMotor.getCurrentPosition()<10){

                    state=autoState.finished;

                }else if (size!=targetAuto.spikes&&cycles>2&&collection.horizontalMotor.getCurrentPosition()<10){
                    state = autoState.collectingSpike;
                    built = building.notBuilt;
                }

            }
            //spike3
        } else if (state == autoState.collectingSpike||state == autoState.collectingSub&&cycles>2.9 ) {

            if (built == building.notBuilt){
                follow.setPath(paths.returnPath("spike1Pickup"));
                targetHeading = 180;
                built = building.built;
                collection.queueCommand(collection.collect);
            }
            if (follow.isFinished()){
                collect=true;
                collectTimer.reset();
                collection.griperRotate.setPosition(45);
                collection.queueCommand(collection.collect);
                if (collect&&collectTimer.milliseconds()>700){
                    collection.queueCommand(collection.transfer);
                    collect=false;
                }


                if (size!=targetAuto.spikes&&cycles>3&&!collect){
                    state = autoState.collectingSpike;
                    built = building.notBuilt;
                }

            }
        } else if ( state == autoState.delivering) {

            if (built == building.notBuilt){
                delivery.queueCommand(delivery.transfer);
                delivery.queueCommand(delivery.slideSetPonts(delivery.highBasket));
                delivery.queueCommand(delivery.deposit);
                follow.setPath(paths.returnPath("spike1Drop"));
                targetHeading = 225;
                drop=true;
                dropTimer.reset();
                built = building.built;

            }
            if (follow.isFinished()){

                if (delivery.fourbarState== Delivery.fourBarState.basketDeposit&&drop&&dropTimer.milliseconds()>4000){

                    delivery.queueCommand(delivery.deposit);

                    delivery.queueCommand(delivery.deposit);

                    delivery.queueCommand(delivery.slideSetPonts(0));

                    drop=false;

                }


                if (size==targetAuto.spikes&&cycles<3.1&&collection.horizontalMotor.getCurrentPosition()<10){

                    state=autoState.finished;

                }else if (size!=targetAuto.spikes&&cycles>3&&collection.horizontalMotor.getCurrentPosition()<10){
                    state = autoState.collectingSpike;
                    built = building.notBuilt;
                }

            }
        }



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
