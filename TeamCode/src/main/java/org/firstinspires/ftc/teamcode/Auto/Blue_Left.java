package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;
import dev.weaponboy.command_library.Subsystems.Delivery;
import dev.weaponboy.command_library.Subsystems.Odometry;
import dev.weaponboy.nexus_pathing.Follower.follower;
import dev.weaponboy.nexus_pathing.PathGeneration.commands.sectionBuilder;
import dev.weaponboy.nexus_pathing.PathGeneration.pathsManager;
import dev.weaponboy.nexus_pathing.PathingUtility.RobotPower;
import dev.weaponboy.nexus_pathing.RobotUtilities.Vector2D;

@Autonomous(name = "Blue Left", group = "Test Autos")
public class Blue_Left extends OpModeEX {

    pathsManager paths = new pathsManager();
    follower follow = new follower();
    double targetHeading;
    double cycles=0;
    boolean drop;
    ElapsedTime dropTimer=new ElapsedTime();
    boolean collect;
    boolean autoQueued;
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
        sub
    }
    public enum spikeState{
        zero,
        one,
        two,
        three,
    }
    public spikeState spike = spikeState.zero;
    public spikeState currentSpike = spikeState.zero;
    public autoState state = autoState.preload;
    public building built = building.notBuilt;
    public targetAuto size = targetAuto.preload;

    private final sectionBuilder[] preloadPath = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(344.3, 263), new Vector2D(282, 272), new Vector2D(324, 322.5)),
    };

    private final sectionBuilder[] spike1Pickup = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(325.3, 324), new Vector2D(300, 315.4), new Vector2D(265, 294)),
    };

    private final sectionBuilder[] spike1Drop = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(267, 303.6), new Vector2D(300, 315.4), new Vector2D(324, 322.5)),
    };

    private final sectionBuilder[] spike2Pickup = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(325.3, 324), new Vector2D(311.7, 333.7), new Vector2D(265, 320)),
    };

    private final sectionBuilder[] spike2Drop = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(267, 330.2), new Vector2D(311.7, 333.7), new Vector2D(324, 322.5)),
    };

    private final sectionBuilder[] spike3Pickup = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(325.3, 324), new Vector2D(256.4, 303), new Vector2D(225.5, 345.2)),
    };

    private final sectionBuilder[] spike3Drop = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(246.7, 331.2), new Vector2D(256.4, 303), new Vector2D(330, 322.5)),
    };

    FtcDashboard dashboard = FtcDashboard.getInstance();

    public Telemetry dashboardTelemetry = dashboard.getTelemetry();

    boolean pathing = false;

    @Override
    public void initEX() {

        FtcDashboard.getInstance().startCameraStream(collection.sampleSorter, 30);

        odometry.startPosition(344.3, 263, 270);

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
    public void init_loop() {
        telemetry.addData("type",size);
        telemetry.addData("cycles",cycles);
        telemetry.update();
        if (gamepad1.b){
            size = targetAuto.spikes;
            currentSpike = spikeState.one;
        }
        if (gamepad1.a){
            size = targetAuto.spikes;
            currentSpike = spikeState.two;
        }
        if (!lastGamepad1.x&& currentGamepad1.x){
            size = targetAuto.spikes;
            currentSpike = spikeState.three;
        }

        super.init_loop();
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


            if (delivery.fourbarState== Delivery.fourBarState.basketDeposit&&drop&&dropTimer.milliseconds()>3500){

                delivery.queueCommand(delivery.deposit);

                delivery.queueCommand(delivery.deposit);

                delivery.queueCommand(delivery.slideSetPonts(0));

                drop=false;

            }

            if (follow.isFinished()&&delivery.fourbarState== Delivery.fourBarState.behindNest){


                if (size==targetAuto.preload && delivery.slideMotor.getCurrentPosition() < 20){

                    System.out.println("X: "+ odometry.X());
                    System.out.println("Y: "+ odometry.Y());
                    System.out.println("Heading: "+ odometry.Heading());

                     state=autoState.finished;

                }else if (size == targetAuto.spikes){

                    System.out.println("X: "+ odometry.X());
                    System.out.println("Y: "+ odometry.Y());
                    System.out.println("Heading: "+ odometry.Heading());

                     state = autoState.collectingSpike;
                     built = building.notBuilt;
                     spike = spikeState.one;

                }

           }

            //spike1
        } else if (state == autoState.collectingSpike && spike == spikeState.one) {

            if (built == building.notBuilt){
                follow.setPath(paths.returnPath("spike1Pickup"));
                targetHeading = 180;
                built = building.built;
//                collection.camera.execute();
                collection.griperRotate.setPosition(90);

                autoQueued = false;
            }

//            if (collect&&collectTimer.milliseconds()>4000){
//                collection.queueCommand(collection.transfer);
//                collect=false;
//                collectionDone = true;
//            }

            if (follow.isFinished() && !collection.nestSensor.isPressed()){

                state = autoState.delivering;
                built = building.notBuilt;

                System.out.println("X: "+ odometry.X());
                System.out.println("Y: "+ odometry.Y());
                System.out.println("Heading: "+ odometry.Heading());
//                collectionDone = false;

            } else if (follow.isFinished() && !autoQueued){
//                collection.griperRotate.setPosition(135);
                collection.queueCommand(collection.autoCollectGlobal);
                autoQueued = true;
            }


        } else if (state == autoState.collectingSpike && spike == spikeState.two) {

            if (built == building.notBuilt){
                follow.setPath(paths.returnPath("spike2Pickup"));
                targetHeading = 180;
                built = building.built;
//                collection.camera.execute();
                collection.griperRotate.setPosition(135);

                autoQueued = false;
            }

//            if (collect&&collectTimer.milliseconds()>4000){
//                collection.queueCommand(collection.transfer);
//                collect=false;
//                collectionDone = true;
//            }

            if (follow.isFinished() && !collection.nestSensor.isPressed()){

                state = autoState.delivering;
                built = building.notBuilt;
//                collectionDone = false;

            } else if (follow.isFinished() && !autoQueued){
//                collection.griperRotate.setPosition(135);
                collection.queueCommand(collection.autoCollectGlobal);
                autoQueued = true;
            }


        } else if (state == autoState.collectingSpike && spike == spikeState.three) {

            if (built == building.notBuilt){
                follow.setPath(paths.returnPath("spike3Pickup"));
                targetHeading = 90;
                built = building.built;
//                collection.camera.execute();
                collection.griperRotate.setPosition(135);

                autoQueued = false;
            }

//            if (collect&&collectTimer.milliseconds()>4000){
//                collection.queueCommand(collection.transfer);
//                collect=false;
//                collectionDone = true;
//            }

            if (follow.isFinished() && !collection.nestSensor.isPressed()){

                state = autoState.delivering;
                built = building.notBuilt;
//                collectionDone = false;

            } else if (follow.isFinished() && !autoQueued){
//                collection.griperRotate.setPosition(135);
                collection.queueCommand(collection.autoCollectGlobal);
                autoQueued = true;
            }


        }else if ( state == autoState.delivering) {

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

                if (delivery.fourbarState== Delivery.fourBarState.basketDeposit && drop && dropTimer.milliseconds()>3500){

                    delivery.queueCommand(delivery.deposit);

                    delivery.queueCommand(delivery.deposit);

                    delivery.queueCommand(delivery.slideSetPonts(0));

                    drop=false;

                }


                if (spike == currentSpike && delivery.slideMotor.getCurrentPosition() < 40 &&delivery.getCurrentCommand() != delivery.followMotionPro&&delivery.fourbarState == Delivery.fourBarState.behindNest){

                    state=autoState.finished;

                }else if (spike == spikeState.one && currentSpike != spikeState.one && delivery.slideMotor.getCurrentPosition()<40){
                    state = autoState.collectingSpike;
                    built = building.notBuilt;
                    spike = spikeState.two;
                }else if (spike == spikeState.two && currentSpike != spikeState.two && delivery.slideMotor.getCurrentPosition()<40){
                    state = autoState.collectingSpike;
                    built = building.notBuilt;
                    spike = spikeState.three;
                }

            }
            //spike3
        }

        if (state==autoState.finished){
            requestOpModeStop();
        }

        if (collection.getCurrentCommand() != collection.autoCollectGlobal){
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

}
