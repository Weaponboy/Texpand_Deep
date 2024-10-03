package dev.weaponboy.command_library.Subsystems;

import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import dev.weaponboy.command_library.CommandLibrary.Commands.Command;
import dev.weaponboy.command_library.CommandLibrary.Commands.Execute;
import dev.weaponboy.command_library.CommandLibrary.Commands.LambdaCommand;
import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;
import dev.weaponboy.command_library.CommandLibrary.Subsystem.SubSystem;
import dev.weaponboy.command_library.Hardware.AxonEncoder;
import dev.weaponboy.command_library.Hardware.MotorEx;
import dev.weaponboy.command_library.Hardware.ServoDegrees;
import dev.weaponboy.command_library.Hardware.motionProfile;

public class Collection2 extends SubSystem {

    // slides
    public MotorEx horizontalMotor = new MotorEx();
    double extendoPower = 0;
    motionProfile profile = new motionProfile(1400, 140, 54, 1720, 0.15);

    public ServoDegrees fourBarMainPivot = new ServoDegrees();
    public ServoDegrees fourBarSecondPivot= new ServoDegrees();
    public ServoDegrees griperRotate= new ServoDegrees();
    public ServoImplEx gripServo;
    public Servo linerRailServo;
    public ServoDegrees nest = new ServoDegrees();

    public AxonEncoder linearPosition = new AxonEncoder();

    /**states*/
    public enum fourBar{
        preCollect,
        collect,
        dropNest,
        transferring,
        stowed
    }

    private fourBar fourBarState = fourBar.stowed;

    public Collection2(OpModeEX opModeEX) {
        registerSubsystem(opModeEX, defaultCommand);
    }

    @Override
    public void init() {
        horizontalMotor.initMotor("horizontalMotor", getOpModeEX().hardwareMap);

        fourBarMainPivot.initServo("fourBarMainPivot", getOpModeEX().hardwareMap);
        fourBarSecondPivot.initServo("fourBarSecondPivot", getOpModeEX().hardwareMap);
        griperRotate.initServo("gripperRotate", getOpModeEX().hardwareMap);
        gripServo = getOpModeEX().hardwareMap.get(ServoImplEx.class, "gripServo");
        linerRailServo  = getOpModeEX().hardwareMap.get(ServoImplEx.class, "linearRailServo");
        nest.initServo("nest", getOpModeEX().hardwareMap);
        linearPosition.init(getOpModeEX().hardwareMap, "axon2");

        fourBarMainPivot.setRange(335);
        fourBarSecondPivot.setRange(new PwmControl.PwmRange(500, 2500), 270);
        griperRotate.setRange(new PwmControl.PwmRange(500, 2500), 270);
        gripServo.setPwmRange(new PwmControl.PwmRange(500, 2500));
        nest.setRange(new PwmControl.PwmRange(500, 2500), 270);

        profile.isVertical(false);

        fourBarMainPivot.setDirection(Servo.Direction.REVERSE);
        fourBarMainPivot.setOffset(36);

        nest.setDirection(Servo.Direction.REVERSE);
        nest.setOffset(5);

        griperRotate.setOffset(-10);
        griperRotate.setPosition(135);

    }

    @Override
    public void execute() {
        executeEX();
    }

    public Command defaultCommand = new LambdaCommand(
            () -> {},
            () -> {
                if (horizontalMotor.getCurrentPosition() < 40){
                    extendoPower = 0;
                }else if (horizontalMotor.getCurrentPosition() > 40){
                    extendoPower = 0.01;
                }
            },
            () -> true
    );

    private final Command collect = new Execute(
            () -> {
                fourBarMainPivot.setPosition(96);
                fourBarSecondPivot.setPosition(6);
                fourBarState = fourBar.collect;
            }
    );



}
