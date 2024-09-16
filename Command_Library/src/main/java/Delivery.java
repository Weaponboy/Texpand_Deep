import dev.weaponboy.command_library.CommandLibrary.Subsystem.SubSystem;

import com.qualcomm.robotcore.hardware.DcMotor;

import dev.weaponboy.command_library.CommandLibrary.Commands.LambdaCommand;

public class Delivery extends SubSystem {
    DcMotor slidemMotor;
    DcMotor slideMotor;

    public LambdaCommand deliveryeup = new LambdaCommand(
            () -> {
                slidemMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                slidemMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            },
            () -> {
                slidemMotor.setPower(0.5);

            },

            () -> slidemMotor.getCurrentPosition() > 2000

    );
    public LambdaCommand deliveryslidedown = new LambdaCommand(
            () -> {
                slidemMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                slidemMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            },
            ()-> {
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
