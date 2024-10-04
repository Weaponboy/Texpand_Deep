//package org.firstinspires.ftc.teamcode.Testing.Teleops;
//
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import java.io.FileWriter;
//import java.io.IOException;
//
//import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;
//
//@TeleOp
//public class TestingSlides extends OpModeEX {
//
//    static FileWriter fWriter;
//
//    static {
//        try {
//            fWriter = new FileWriter("/sdcard/FIRST/testWrite.txt");
//        } catch (IOException e) {
//            throw new RuntimeException(e);
//        }
//    }
//
//    @Override
//    public void initEX() {
//
//    }
//
//    @Override
//    public void loopEX() {
//
//        if (currentGamepad1.y && !lastGamepad1.y){
//            try {
//                delivery.genProfile(delivery.lowBasket, fWriter);
//                fWriter.flush();
//            } catch (IOException e) {
//                throw new RuntimeException(e);
//            }
//        }
//
//        System.out.println("loopTime: " + loopTime);
//
//    }
//}
