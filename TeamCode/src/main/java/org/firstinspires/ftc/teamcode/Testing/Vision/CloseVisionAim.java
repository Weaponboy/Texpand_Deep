//package org.firstinspires.ftc.teamcode.Testing.Vision;
//
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;
//import dev.weaponboy.command_library.Subsystems.Collection;
//
//@TeleOp
//public class CloseVisionAim extends OpModeEX {
//
//    @Override
//    public void initEX() {
//
//        collection.camera.execute();
//
//    }
//
//    @Override
//    public void loopEX() {
//
//        System.out.println("camera state: " + collection.portal.getCameraState());
//
////        if (Math.abs(sampleSorter.getAngleRotate()) < 70){
////            collection.griperRotate.setPosition(90+sampleSorter.getAngleRotate());
////        }else {
////            collection.griperRotate.setPosition(90);
////        }
////
//////
//        if (currentGamepad1.a && !lastGamepad1.a){
//
//            if (collection.sampleSorter.getRailTarget() > 0 && collection.sampleSorter.getRailTarget() < 20){
//                collection.setRailTargetPosition(collection.sampleSorter.getRailTarget());
//            }
//
//            double newSlidesTarget = collection.horizontalMotor.getCurrentPosition() / ((double) 440 /35) + collection.sampleSorter.getSlidesDelta();
//
//            if (newSlidesTarget > 0 && newSlidesTarget < 35){
//                if (collection.sampleSorter.getSlidesDelta() > 0){
//                    collection.queueCommand(collection.cameraSetPoint(((collection.horizontalMotor.getCurrentPosition() / ((double) 440 /35)) + collection.sampleSorter.getSlidesDelta()+3)));
//                }else {
//                    collection.queueCommand(collection.cameraSetPoint(((collection.horizontalMotor.getCurrentPosition() / ((double) 440 /35)) + collection.sampleSorter.getSlidesDelta()+3)));
//                }
////                collection.queueCommand(collection.collect);
//            }
//
//            collection.griperRotate.setPosition(90+collection.sampleSorter.getAngleRotate());
//        }
//
//        if (currentGamepad1.left_bumper && !lastGamepad1.left_bumper){
//
//            double railTarget = collection.sampleSorter.getRailTarget()-10;
//            railTarget = collection.getRailPosition()+railTarget;
//
//            if (railTarget > 0 && railTarget < 20){
//                collection.setRailTargetPosition(railTarget);
//            }
//
//            double newSlidesTarget = collection.horizontalMotor.getCurrentPosition() / ((double) 440 /35) + collection.sampleSorter.getSlidesDelta();
//
//            if (newSlidesTarget > 0 && newSlidesTarget < 35){
//                if (collection.sampleSorter.getSlidesDelta() > 0){
//                    collection.queueCommand(collection.collect(((collection.horizontalMotor.getCurrentPosition() / ((double) 440 /35)) + collection.sampleSorter.getSlidesDelta())));
//                }else {
//                    collection.queueCommand(collection.collect(((collection.horizontalMotor.getCurrentPosition() / ((double) 440 /35)) + collection.sampleSorter.getSlidesDelta())));
//                }
////                collection.queueCommand(collection.collect);
//            }
//
//            collection.griperRotate.setPosition(90+collection.sampleSorter.getAngleRotate());
//        }
//
//
//        if (gamepad1.right_bumper){
//            collection.queueCommand(collection.collect(0));
//
//        }
//
//        if (currentGamepad1.back && !(lastGamepad1.back) && collection.getClawsState() == Collection.clawState.drop){
//            collection.setClawsState(Collection.clawState.grab);
//        } else if (currentGamepad1.back && !(lastGamepad1.back) && collection.getClawsState() == Collection.clawState.grab) {
//            collection.setClawsState(Collection.clawState.drop);
//        }
//
//        if (currentGamepad1.x && !lastGamepad1.x){
//            collection.setRailTargetPosition(10);
//        }
////
//        if (currentGamepad1.b && !lastGamepad1.b){
//            collection.queueCommand(collection.collect);
//        }
//
//        if (currentGamepad1.y && !lastGamepad1.y){
//            collection.camera.execute();
//        }
////
//
//        double newSlidesTarget = collection.horizontalMotor.getCurrentPosition() / ((double) 440 /35) + collection.sampleSorter.getSlidesDelta();
//        telemetry.addData("horizontal", collection.horizontalMotor.getCurrentPosition()/(440/35));
//        telemetry.addData("sampleSorter.getAngleRotate()", collection.sampleSorter.getAngleRotate());
//        telemetry.addData("getRailTarget", collection.sampleSorter.getRailTarget());
//        telemetry.addData("collection.horizontalMotor.getCurrentPosition()", collection.sampleSorter.getSlidesDelta());
//        telemetry.addData("newSlidesTarget", newSlidesTarget);
//        telemetry.update();
//
//    }
//
//    @Override
//    public void stop() {
//
//    }

//}
