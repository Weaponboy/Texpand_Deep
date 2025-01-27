//package org.firstinspires.ftc.teamcode.Auto.Red;
//
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;
//import dev.weaponboy.command_library.Subsystems.Collection;
//import dev.weaponboy.command_library.Subsystems.Delivery;
//import dev.weaponboy.nexus_pathing.Follower.follower;
//import dev.weaponboy.nexus_pathing.PathGeneration.commands.sectionBuilder;
//import dev.weaponboy.nexus_pathing.PathGeneration.pathBuilder;
//import dev.weaponboy.nexus_pathing.PathGeneration.pathsManager;
//import dev.weaponboy.nexus_pathing.PathingUtility.RobotPower;
//import dev.weaponboy.nexus_pathing.RobotUtilities.Vector2D;
//import dev.weaponboy.vision.SamplePipelines.findAngleUsingContour;
//
//@Autonomous(name = "Blue Right", group = "Blue Autos")
//public class Blue_Right_Full_Auto extends OpModeEX {
//
//    double targetHeading;
//
//    pathsManager paths = new pathsManager();
//    follower follow = new follower();
//
//    private final sectionBuilder[] preloadDelivery = {
//            () -> paths.addPoints(new Vector2D(339, 160), new Vector2D(252, 178))
//    };
//
//    private final sectionBuilder[] obs_collecting = {
//            () -> paths.addPoints(new Vector2D(250, 180), new Vector2D(280, 190), new Vector2D(332,122))
//    };
//
//    private final sectionBuilder[] clip_And_Collect_One = {
//            () -> paths.addPoints(new Vector2D(332, 122), new Vector2D(280,190),new Vector2D(250, 182))
//    };
//
//    private final sectionBuilder[] clip_And_Collect_Two = {
//            () -> paths.addPoints(new Vector2D(332, 122), new Vector2D(280,190),new Vector2D(250, 174))
//    };
//
//    private final sectionBuilder[] clip_And_Collect_Three = {
//            () -> paths.addPoints(new Vector2D(332, 122), new Vector2D(280,190),new Vector2D(250, 167))
//    };
//
//    public enum autoState {
//        preLoad,
//        cycle_one,
//        cycle_two,
//        cycle_three,
//        cycle_four,
//        final_Clip,
//        finished;
//
//        public static autoState next(autoState current) {
//            autoState[] values = autoState.values();
//            int nextIndex = (current.ordinal() + 1) % values.length; // Wrap around to the first enum
//            return values[nextIndex];
//        }
//    }
//
//    public enum CycleState{
//        clip_and_collect,
//        obs_collect
//    }
//
//    public enum building {
//        built,
//        notBuilt
//    }
//
//    boolean following = false;
//    boolean collectSample = false;
//    boolean clipped = false;
//
//    autoState state = autoState.preLoad;
//    autoState targetState = autoState.cycle_two;
//    building built = building.notBuilt;
//    building cycleBuilt = building.notBuilt;
//    CycleState cycleState = CycleState.clip_and_collect;
//
//    boolean transferring = false;
//    ElapsedTime transferringWait = new ElapsedTime();
//
//    ElapsedTime extendoWait = new ElapsedTime();
//
//    boolean runCollect = false;
//
//    @Override
//    public void initEX() {
//
//        delivery.setGripperState(Delivery.gripper.grab);
//
//        delivery.griperSev.setPosition(78);
//
//        odometry.startPosition(342.5, 164, 180);
//
//        paths.addNewPath("preloadPath");
//        paths.buildPath(preloadDelivery);
//
//        paths.addNewPath("obs_collecting");
//        paths.buildPath(obs_collecting);
//
//        paths.addNewPath("clip_One");
//        paths.buildPath(clip_And_Collect_One);
//
//        paths.addNewPath("clip_Two");
//        paths.buildPath(clip_And_Collect_Two);
//
//        paths.addNewPath("clip_Three");
//        paths.buildPath(clip_And_Collect_Three);
//
//        follow.setPath(paths.returnPath("preloadPath"));
//
//        FtcDashboard.getInstance().startCameraStream(collection.sampleDetector, 30);
//
//        collection.sampleDetector.setScanning(false);
//        collection.sampleDetector.setTargetColor(findAngleUsingContour.TargetColor.blue);
//        collection.sampleDetector.closestFirst = true;
//    }
//
//    @Override
//    public void loopEX() {
//
//        if (state == autoState.preLoad) {
//
//            if (built == building.notBuilt) {
//                follow.setPath(paths.returnPath("preloadPath"));
//                targetHeading = 180;
//                following = true;
//                clipped = false;
//                runCollect = false;
//                built = building.built;
//
//                delivery.queueCommand(delivery.preClipFront);
//            }
//
//            if (!follow.isFinished() && Math.abs(odometry.getXVelocity()) < 2 && follow.getXError() < 5){
//                follow.finishPath();
//                following = false;
//            }
//
//            if (targetState == autoState.final_Clip){
//
//                if (follow.isFinished() && delivery.getCurrentCommand() != delivery.preClipFront && !collection.getChamberCollect() && !clipped){
//                    delivery.queueCommand(delivery.clipFront);
//                    clipped = true;
//                }
//
//                if (follow.isFinished() && delivery.getSlidePositionCM() < 5 && collection.getSlidePositionCM() < 2) {
//                    state = autoState.final_Clip;
//                    built = building.notBuilt;
//                    runCollect = false;
//                    cycleBuilt = building.notBuilt;
//                }
//
//            }else{
//                if (follow.isFinished() && delivery.getCurrentCommand() != delivery.preClipFront && !collection.getChamberCollect()){
//
//                    runCollect = true;
//                    collection.sampleDetector.setScanning(true);
//                    collection.portal.resumeStreaming();
//
//                    boolean detecting = true;
//                    int counter = 0;
//
//                    while (detecting && counter < 20){
//                        counter++;
//
//                        if (!collection.sampleDetector.detections.isEmpty() && counter > 10){
//                            collection.sampleDetector.setScanning(false);
//                            collection.portal.stopStreaming();
//                            collection.sampleMap = collection.sampleDetector.convertPositionsToFieldPositions(new RobotPower(odometry.X(), odometry.Y(), odometry.Heading()), delivery.getSlidePositionCM(), 180 - (90 -Math.abs((delivery.mainPivot.getPositionDegrees()-190.5)*1.2587)));
//                            detecting = false;
//                        }else {
//                            try {
//                                Thread.sleep(50);
//                            } catch (InterruptedException e) {
//                                throw new RuntimeException(e);
//                            }
//                        }
//                    }
//
//                    collection.queueCommand(collection.autoCollectChamber);
//                    delivery.queueCommand(delivery.preClipFront);
//                    delivery.queueCommand(delivery.clipFront);
//                    collection.setChamberCollect(true);
//
//                }else if (delivery.getSlidePositionCM() > 15 && !runCollect){
//                    delivery.mainPivot.setPosition(delivery.findCameraScanPosition(false));
//                    delivery.secondPivot.setPosition(80);
//                }
//
//                if (follow.isFinished() && delivery.getSlidePositionCM() < 5 && collection.getSlidePositionCM() < 5.5 && delivery.getCurrentCommand() != delivery.preClipFront && collection.getChamberCollect() && collection.breakBeam.isPressed() && collection.fourBarMainPivot.getPositionDegrees() > 90) {
//                    state = autoState.cycle_one;
//                    built = building.notBuilt;
//                    runCollect = false;
//                    cycleBuilt = building.notBuilt;
//                }
//            }
//
//        }else if (state == autoState.cycle_one || state == autoState.cycle_two || state == autoState.cycle_three || state == autoState.cycle_four) {
//
//            if (built == building.notBuilt){
//                cycleState = CycleState.obs_collect;
//                built = building.built;
//            }
//
//            fullCycle();
//
//        }else if (state == autoState.final_Clip){
//
//            if (built == building.notBuilt){
//                cycleState = CycleState.obs_collect;
//                built = building.built;
//                cycleBuilt = building.notBuilt;
//            }
//
//            if (cycleState == CycleState.obs_collect) {
//
//                if (cycleBuilt == building.notBuilt) {
//                    cycleBuilt = building.built;
//                    follow.setPath(paths.returnPath("obs_collecting"));
//                    following = true;
//                    collectSample = false;
//                    targetHeading = 270;
//                }
//
//                if (odometry.Heading() > 240 && !follow.isFinished(2,2) && !collectSample){
//                    collection.targetPointWithExtendo(new Vector2D(332.5, 61));
//                    collection.griperRotate.setPosition(180);
//                }
//
//                if (follow.isFinished(2,2) && !collectSample && collection.horizontalMotor.getVelocity() < 10){
//                    following = false;
//                    collection.queueCommand(collection.obs_Collect_No_Drop);
//                    collectSample = true;
//                } else if (follow.isFinished(2,2) && collectSample && collection.getSlideTarget() == 0){
//                    cycleState = CycleState.clip_and_collect;
//                    cycleBuilt = building.notBuilt;
//                }
//
//            }else if (cycleState == CycleState.clip_and_collect) {
//
//                if (cycleBuilt == building.notBuilt) {
//                    cycleBuilt = building.built;
//                    follow.setPath(paths.returnPath("clip_Three"));
//                    following = true;
//                    clipped = false;
//                    runCollect = false;
//
//                    targetHeading = 180;
//                }
//
//                if (!follow.isFinished() && Math.abs(odometry.getXVelocity()) < 2 && follow.getXError() < 5){
//                    following = false;
//                    follow.finishPath();
//                }
//
//                if (collection.breakBeam.isPressed() && collection.getFourBarState() == Collection.fourBar.transferUp && !transferring && collection.slidesReset.isPressed()){
//
//                    delivery.queueCommand(delivery.transfer);
//
//                    delivery.queueCommand(collection.transferDrop);
//
//                    delivery.queueCommand(delivery.transfer);
//
//                    transferring = true;
//
//                } else if (!collection.breakBeam.isPressed() && transferring && delivery.getCurrentCommand() != delivery.transfer) {
//                    transferring = false;
//                    delivery.queueCommand(delivery.preClipFront);
//                }
//
//                if (follow.isFinished() && delivery.getCurrentCommand() != delivery.preClipFront && !transferring && !clipped){
//                    delivery.queueCommand(delivery.preClipFront);
//                    delivery.queueCommand(delivery.clipFront);
//                    clipped = true;
//                }
//
//                if (follow.isFinished(2, 2) && clipped && delivery.getCurrentCommand() != delivery.preClipFront && delivery.getSlidePositionCM() < 4) {
//                    state = autoState.finished;
//                }
//
//            }
//        }
//
//        if (state == autoState.finished) {
//            requestOpModeStop();
//        }
//
//        odometry.queueCommand(odometry.updateLineBased);
//
//        if (following) {
//            RobotPower currentPower = follow.followPathAuto(targetHeading, odometry.Heading(), odometry.X(), odometry.Y(), odometry.getXVelocity(), odometry.getYVelocity());
//            telemetry.addData("Loop time", loopTime);
//            telemetry.addData("Y", odometry.Y());
//            telemetry.addData("Heading", odometry.Heading());
//            telemetry.addData("X", odometry.X());
//            telemetry.addData("getVertical", currentPower.getVertical());
//            telemetry.addData("getHorizontal", currentPower.getHorizontal());
//            telemetry.addData("getPivot", currentPower.getPivot());
//            telemetry.update();
//
//            driveBase.queueCommand(driveBase.drivePowers(currentPower));
//        }else {
//            driveBase.queueCommand(driveBase.drivePowers(new RobotPower(0, 0,follow.getTurnPower(targetHeading, odometry.Heading()))));
//        }
//    }
//
//    public void fullCycle(){
//
//        if (cycleState == CycleState.obs_collect) {
//
//            if (cycleBuilt == building.notBuilt) {
//                cycleBuilt = building.built;
//                follow.setPath(paths.returnPath("obs_collecting"));
//                following = true;
//                collectSample = false;
//                targetHeading = 270;
//            }
//
//            if (odometry.Heading() > 240 && !follow.isFinished(2,2) && !collectSample){
//                collection.targetPointWithExtendo(new Vector2D(338, 64));
//                collection.griperRotate.setPosition(180);
//            }
//
//            if (follow.isFinished(2,2) && !collectSample && collection.horizontalMotor.getVelocity() < 10){
//                following = false;
//                collection.queueCommand(collection.obs_Collect);
//                collectSample = true;
//            } else if (follow.isFinished(2,2) && collectSample && collection.getSlideTarget() == 0){
//                cycleState = CycleState.clip_and_collect;
//                cycleBuilt = building.notBuilt;
//            }
//
//        }else if (cycleState == CycleState.clip_and_collect) {
//
//            if (cycleBuilt == building.notBuilt) {
//                cycleBuilt = building.built;
//                follow.setPath(getClipPath());
//                following = true;
//                clipped = false;
//                runCollect = false;
//
//                targetHeading = 180;
//            }
//
//            if (!follow.isFinished() && Math.abs(odometry.getXVelocity()) < 2 && follow.getXError() < 5){
//                following = false;
//                follow.finishPath();
//            }
//
//            if (collection.breakBeam.isPressed() && collection.getFourBarState() == Collection.fourBar.transferUp && !transferring && collection.slidesReset.isPressed()){
//
//                delivery.queueCommand(delivery.transfer);
//
//                delivery.queueCommand(collection.transferDrop);
//
//                delivery.queueCommand(delivery.transfer);
//
//                transferring = true;
//
//            } else if (!collection.breakBeam.isPressed() && transferring && delivery.getCurrentCommand() != delivery.transfer) {
//                transferring = false;
//                if (state == targetState){
//                    delivery.queueCommand(delivery.preClipFront);
//                }else{
//                    delivery.queueCommand(delivery.cameraScan);
//                }
//            }
//
//            if (state == targetState){
//
//                if (follow.isFinished() && delivery.getCurrentCommand() != delivery.preClipFront && !clipped && !transferring){
//                    delivery.queueCommand(delivery.clipFront);
//                    clipped = true;
//                }
//
//                if (follow.isFinished(2, 2) && clipped && delivery.getCurrentCommand() != delivery.preClipFront && delivery.getSlidePositionCM() < 4) {
//                    state = autoState.final_Clip;
//                    built = building.notBuilt;
//                }
//
//            }else {
//
//                if (follow.isFinished() && delivery.getCurrentCommand() != delivery.preClipFront && !collection.getChamberCollect()){
//
//                    delivery.mainPivot.setPosition(delivery.findCameraScanPosition(true));
//
//                    runCollect = true;
//                    collection.sampleDetector.setScanning(true);
//                    collection.portal.resumeStreaming();
//
//                    boolean detecting = true;
//                    int counter = 0;
//
//                    while (detecting && counter < 20){
//                        counter++;
//
//                        if (!collection.sampleDetector.detections.isEmpty() && counter > 5){
//                            collection.sampleDetector.setScanning(false);
//                            collection.portal.stopStreaming();
//                            collection.sampleMap = collection.sampleDetector.convertPositionsToFieldPositions(new RobotPower(odometry.X(), odometry.Y(), odometry.Heading()), delivery.getSlidePositionCM(), 180 - (90 -Math.abs((delivery.mainPivot.getPositionDegrees()-190.5)*1.2587)));
//                            detecting = false;
//                        }else {
//                            try {
//                                Thread.sleep(50);
//                            } catch (InterruptedException e) {
//                                throw new RuntimeException(e);
//                            }
//                        }
//                    }
//
//                    collection.queueCommand(collection.autoCollectChamber);
//                    delivery.queueCommand(delivery.preClipFront);
//                    delivery.queueCommand(delivery.clipFront);
//                    collection.setChamberCollect(true);
//
//                }else if (delivery.getSlidePositionCM() > 15 && !runCollect){
//                    delivery.mainPivot.setPosition(delivery.findCameraScanPosition(true));
//                    delivery.secondPivot.setPosition(80);
//                }
//
//                if (follow.isFinished(2, 2) && runCollect && delivery.getSlidePositionCM() < 5 && collection.getSlidePositionCM() < 5.5 && delivery.getCurrentCommand() != delivery.preClipFront && collection.getChamberCollect() && collection.breakBeam.isPressed() && collection.fourBarMainPivot.getPositionDegrees() > 90) {
//                    state = autoState.next(state);
//                    built = building.notBuilt;
//                    cycleBuilt = building.notBuilt;
//                }
//
//            }
//
//        }
//
//    }
//
//    public pathBuilder getClipPath(){
//
//        if (state == autoState.cycle_one){
//            return paths.returnPath("clip_One");
//        }else if (state == autoState.cycle_two){
//            return paths.returnPath("clip_Two");
//        }else if (state == autoState.cycle_three){
//            return paths.returnPath("clip_Three");
//        }else {
//            return paths.returnPath("preloadPath");
//        }
//
//    }
//}
