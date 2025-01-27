package org.firstinspires.ftc.teamcode.Testing.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Point;

import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;
import dev.weaponboy.nexus_pathing.Follower.follower;
import dev.weaponboy.nexus_pathing.PathGeneration.commands.sectionBuilder;
import dev.weaponboy.nexus_pathing.PathGeneration.pathsManager;
import dev.weaponboy.nexus_pathing.RobotUtilities.Vector2D;

@Autonomous
public class TestingTransferBug extends OpModeEX {

    pathsManager paths = new pathsManager();

    follower follow = new follower();

    double targetHeading;

    boolean drop;
    ElapsedTime dropTimer=new ElapsedTime();
    boolean collect = false;
    boolean autoQueued = false;
    boolean pullDownSlides = false;
    ElapsedTime dropTimerDriving = new ElapsedTime();
    boolean stop = false;

    double vertical = 0;

    boolean headingOverride = false;

    public enum autoState{
        preload,
        spikeOne,
        spikeTwo,
        spikeThree,
        one,
        two,
        three,
        four,
        five,
        finished;

        public static autoState next(autoState current) {
            autoState[] values = autoState.values();
            int nextIndex = (current.ordinal() + 1) % values.length; // Wrap around to the first enum
            return values[nextIndex];
        }
    }

    public enum cycleState{
        spikeCollect,
        subCollect,
        basketDrob
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

    boolean busyDetecting = false;
    ElapsedTime detectionTimer = new ElapsedTime();
    int counter = 0;

    boolean pathing = false;
    boolean headingAdjustment = false;

    public cycleState CycleState = cycleState.basketDrob;

    public autoState targetState = autoState.three;
    public autoState state = autoState.preload;
    public building built = building.notBuilt;
    public building cycleBuilt = building.notBuilt;

    private final sectionBuilder[] preloadPath = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(344.3, 275), new Vector2D(322, 282), new Vector2D(332, 332)),
    };

    private final sectionBuilder[] spikeOne = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(334, 334), new Vector2D(311, 326)),
    };

    private final sectionBuilder[] spikeTwo = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(334, 334), new Vector2D(323, 311), new Vector2D(292, 326)),
    };

    private final sectionBuilder[] subCollect = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(326.3, 326), new Vector2D(200, 295), new Vector2D(200, 240)),
    };

    private final sectionBuilder[] spikeDeposit = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(200, 232), new Vector2D(204, 288), new Vector2D(330, 330)),
    };

    private final sectionBuilder[] subDeposit = new sectionBuilder[]{
            () -> paths.addPoints(new Vector2D(200, 232), new Vector2D(204, 288), new Vector2D(327, 329)),
    };

    FtcDashboard dashboard = FtcDashboard.getInstance();

    public Telemetry dashboardTelemetry = dashboard.getTelemetry();

    @Override
    public void initEX() {

        odometry.startPosition(0, 0, 180);

        paths.addNewPath("preloadPath");

        paths.buildPath(preloadPath);

        follow.setPath(paths.returnPath("preloadPath"));

        paths.addNewPath("collectSub");

        paths.buildPath(subCollect);

        follow.setPath(paths.returnPath("collectSub"));

        paths.addNewPath("dropBasket");

        paths.buildPath(subDeposit);

        paths.addNewPath("dropBasketSpike");

        paths.buildPath(spikeDeposit);

        follow.setPath(paths.returnPath("dropBasket"));

        paths.addNewPath("spikeOne");

        paths.buildPath(spikeOne);

        paths.addNewPath("spikeTwo");

        paths.buildPath(spikeTwo);

        FtcDashboard.getInstance().startCameraStream(collection.sampleDetector, 30);

        collection.sampleDetector.closestFirst = true;

        collection.setCancelTransfer(false);

    }

    @Override
    public void loopEX() {

        if (state == autoState.preload) {

            if (built == building.notBuilt) {

                built = building.built;

                collection.queueCommand(collection.collect);

                //place sample 30cm in front of the robot
                collection.queueCommand(collection.extendoTargetPoint(new Vector2D(47.5, 0)));

                collection.queueCommand(collection.collect);

                collection.queueCommand(collection.transferAuto);

                collection.queueCommand(delivery.transfer);

                collection.queueCommand(collection.transferDropAuto);

                collection.queueCommand(delivery.closeGripper);

                collection.queueCommand(collection.openGripper);
            }

        }

        telemetry.addData("Position", collection.getSlidePositionCM());
        telemetry.addData("Target", collection.getSlideTarget());
        telemetry.addData("Loop time", loopTime);
        telemetry.update();

    }

}
