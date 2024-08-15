package dev.weaponboy.command_library.Examples.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import dev.weaponboy.command_library.Commands.LambdaCommand;
import dev.weaponboy.command_library.OpmodeEX.OpModeEX;
import dev.weaponboy.command_library.Subsystem.SubSystem;

public class DriveBase extends SubSystem {

    DcMotorEx slideMotor;

    public DriveBase(OpModeEX opModeEX){
        registerSubsystem(opModeEX, PIDControl);
    }

    @Override
    public void init() {
        slideMotor = getOpModeEX().hardwareMap.get(DcMotorEx.class, "motor1");
    }

    @Override
    public void execute() {
        executeEX();
    }

    public LambdaCommand PIDControl = new LambdaCommand(
            () -> {
                slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            },
            () -> slideMotor.setPower(0.5),
            () -> slideMotor.getCurrentPosition() > 2000

    );

}
