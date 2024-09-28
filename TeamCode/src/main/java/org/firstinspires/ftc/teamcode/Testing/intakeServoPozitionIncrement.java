package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;
@TeleOp
public class intakeServoPozitionIncrement extends OpModeEX {
    Servo fourBarMainPivot;
    Servo fourBarSecondPivot;
    Servo griperRotate;
    Servo linerRailServo;
    double mainPivotPower;
    double seconPivotPower;
    double griperRotatePower;

    @Override
    public void initEX() {

//        fourBarMainPivot=hardwareMap.get(Servo.class,"fourBarMainPivot");
//        fourBarSecondPivot=hardwareMap.get(Servo.class,"fourBarSecondPivot");
//        griperRotate=hardwareMap.get(Servo.class,"griperRotate");
//        linerRailServo=hardwareMap.get(Servo.class,"linerRailServo");

//          delivery.mainPivot.setPosition(60);

//        delivery.secondPivot.setPosition(210);

//        delivery.linierRail.setPosition(270);

//        delivery.griperSev.setPosition(110);

 //       colection.gripServo.setPosition(0.1);

//        colection.griperRotate.setPosition(180);

    //    colection.fourBarSecondPivot.setPosition(100);

//        colection.fourBarMainPivot.setDirection(Servo.Direction.REVERSE);
//
//        colection.fourBarMainPivot.setOffset(36);
//
//        colection.nest.setDirection(Servo.Direction.REVERSE);
//
//        colection.nest.setOffset(5);
//
//        colection.nest.setPosition(135);
//
//        colection.griperRotate.setOffset(-10);
//
//        colection.griperRotate.setPosition(135);
//
//        colection.fourBarMainPivot.setPosition(120);
//
//        colection.fourBarSecondPivot.setPosition(55);

        colection.preCollect.execute();
        delivery.mainPivot.setPosition(65);
        delivery.secondPivot.setPosition(260);
    }

    @Override
    public void loopEX() {

//        delivery.queueCommand(delivery.drop);

//          colection.gripServo.setPosition((colection.gripServo.getPosition()*180) + 0.5);

//        if (gamepad1.a) {
//            mainPivotPower =fourBarMainPivot.getPosition()+ 1;
//            fourBarMainPivot.setPosition(mainPivotPower);
//        }
//         if (gamepad1.b) {
//            seconPivotPower=fourBarSecondPivot.getPosition()+ 1;
//            fourBarSecondPivot.setPosition(seconPivotPower);
//        }
 //       if (gamepad1.x) {
     //       griperRotatePower=griperRotate.getPosition()+ 1;
     //       griperRotate.setPosition(griperRotatePower);
     //   }
        if (currentGamepad1.y && !lastGamepad1.y) {
            delivery.queueCommand(delivery.transfer);
        }
        if (currentGamepad1.x && !lastGamepad1.x) {
            delivery.queueCommand(delivery.postTransfer);
        }

       if (currentGamepad1.a && !lastGamepad1.a) {
           delivery.queueCommand(delivery.behindTransfer);
        }

  //      if (currentGamepad1.b && !lastGamepad1.b) {
    //        delivery.queueCommand(delivery.deposit);
     //   }

       if (gamepad1.start) {
           delivery.queueCommand(delivery.grip);
        }

        if (gamepad1.back) {
            delivery.queueCommand(delivery.drop);
        }

////        try {
////            Thread.sleep(50);
////        } catch (InterruptedException e) {
////            throw new RuntimeException(e);
//        }

        telemetry.addData("linear servo position", colection.linearPosition.getPosition());
        telemetry.addData("secondPivotPozition",seconPivotPower);
        telemetry.addData("griperRotatePozition",loopTime);
        telemetry.update();
    }
}
