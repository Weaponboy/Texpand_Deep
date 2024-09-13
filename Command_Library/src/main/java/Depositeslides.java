import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;
import dev.weaponboy.command_library.CommandLibrary.Subsystem.SubSystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import dev.weaponboy.command_library.CommandLibrary.Commands.LambdaCommand;
import dev.weaponboy.command_library.CommandLibrary.Subsystem.SubSystem;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
public class Depositeslides extends SubSystem {
    DcMotor slidemMotor;
    DcMotor slideMotor;

    public LambdaCommand depositslideup = new LambdaCommand(
            () -> {
                slidemMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                slidemMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            },
            () -> {
                slidemMotor.setPower(0.5);
                slideMotor.setPower(0.5);
            },

            () -> slidemMotor.getCurrentPosition() > 2000 &&
                    slideMotor.getCurrentPosition() > 2000
    );
    public LambdaCommand depositslidedown = new LambdaCommand(
            () -> {
                slidemMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                slidemMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            },
            ()-> {
                slidemMotor.setPower(-0.5);
                slidemMotor.setPower(-0.5);
            },
            ()-> slidemMotor.getCurrentPosition() < 1000
    );

    @Override
    public void init() {


    }

    @Override
    public void execute() {

    }
}
