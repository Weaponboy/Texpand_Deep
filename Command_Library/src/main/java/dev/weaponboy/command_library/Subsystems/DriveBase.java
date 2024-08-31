package dev.weaponboy.command_library.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import dev.weaponboy.command_library.CommandLibrary.Commands.Command;
import dev.weaponboy.command_library.CommandLibrary.Commands.LambdaCommand;
import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;
import dev.weaponboy.command_library.CommandLibrary.Subsystem.SubSystem;

public class DriveBase extends SubSystem {
    DcMotorEx leftFD;
    DcMotorEx rightFD;
    DcMotorEx backRD;
    DcMotorEx backLD;

    double vertikal ;
    double turn ;
    double strafe;
    public DriveBase(OpModeEX opModeEX){

        registerSubsystem(opModeEX,driveCommand);
    }

    @Override
    public void init() {
        leftFD = getOpModeEX().hardwareMap.get(DcMotorEx.class, "leftFD");
        rightFD = getOpModeEX().hardwareMap.get(DcMotorEx.class, "rightFD");
        backLD = getOpModeEX().hardwareMap.get(DcMotorEx.class, "backLD");
        backRD = getOpModeEX().hardwareMap.get(DcMotorEx.class, "backRD");
    }

    @Override
    public void execute() {
        executeEX();
    }

    public Command drivePowers (double vertikal, double turn, double strafe){
        this.vertikal =vertikal;
        this.strafe =strafe;
        this.turn =turn;

        return driveCommand;
    }
    LambdaCommand driveCommand = new LambdaCommand(
            () -> {
                System.out.println("init");
            },
            () -> {
                leftFD.setPower(vertikal-strafe-turn);
                rightFD.setPower(vertikal+strafe+turn);
                backLD.setPower(vertikal+strafe-turn);
                backRD.setPower(vertikal-strafe+turn);
            },
            () -> true
    );
}
