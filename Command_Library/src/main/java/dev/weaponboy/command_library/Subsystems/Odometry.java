package dev.weaponboy.command_library.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;

import dev.weaponboy.command_library.Subsystems.GoBildaPinpointDriver.Register;
import dev.weaponboy.command_library.CommandLibrary.Commands.Command;
import dev.weaponboy.command_library.CommandLibrary.Commands.LambdaCommand;
import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;
import dev.weaponboy.command_library.CommandLibrary.Subsystem.SubSystem;
import dev.weaponboy.command_library.Hardware.DistanceSensor;
import dev.weaponboy.command_library.Hardware.SensorReadings;

public class Odometry extends SubSystem {

    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer

    double oldTime = 0;

    Register[] onlyPosition = {
            Register.DEVICE_STATUS,
            Register.X_POSITION,
            Register.Y_POSITION,
            Register.H_ORIENTATION,
            Register.X_VELOCITY,
            Register.Y_VELOCITY,
    };

    DcMotorEx leftPod;
    DcMotorEx rightPod;
    DcMotorEx backPod;

    double X, Y, Heading;
    double startX, startY, startHeading;

    double XVelocity = 0;
    double YVelocity = 0;

    public Odometry(OpModeEX opModeEX) {
        registerSubsystem(opModeEX, update);
    }

    public void startPosition(double X, double Y, int Heading){
        this.X = X;
        this.Y = Y;
        this.Heading = Math.toRadians(Heading);
    }

    @Override
    public void init() {
        /*
         * Initialize the hardware variables. Note that the strings used here must correspond
         * to the names assigned during the robot configuration step on the DS or RC devices.
         */
        odo = getOpModeEX().hardwareMap.get(GoBildaPinpointDriver.class,"odo");

        //Here we set the bulk read scope we created earlier.
        odo.setBulkReadScope(onlyPosition);

        /*
         Another new feature to the Pinpoint is on-device error detection. This allows the device
         to ensure that the most recent read hasn't been corrupted. We use CRC8 error detection
         which is a lightweight, and accurate option. The sending device does a polynomial division
         on the contents of the transmission, and sends the result of that calculation along with
         the data. The receiving device then repeats that calculation and compares it. If the two
         results do not match, then the previous read of that data is repeated, and a
         "FAULT_BAD_READ" flag is thrown.
         */
        odo.setErrorDetectionType(GoBildaPinpointDriver.ErrorDetectionType.CRC);

        /*
        Set the odometry pod positions relative to the point that the odometry computer tracks around.
        The X pod offset refers to how far sideways from the tracking point the
        X (forward) odometry pod is. Left of the center is a positive number,
        right of center is a negative number. the Y pod offset refers to how far forwards from
        the tracking point the Y (strafe) odometry pod is. forward of center is a positive number,
        backwards is a negative number.
         */
        odo.setOffsets(-84.0, -168.0, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1

        /*
        Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
        the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.
        If you're using another kind of odometry pod, uncomment setEncoderResolution and input the
        number of ticks per mm of your odometry pod.
         */
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        //odo.setEncoderResolution(13.26291192, DistanceUnit.MM);


        /*
        Set the direction that each of the two odometry pods count. The X (forward) pod should
        increase when you move the robot forward. And the Y (strafe) pod should increase when
        you move the robot to the left.
         */
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);


        /*
        Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
        The IMU will automatically calibrate when first powered on, but recalibrating before running
        the robot is a good idea to ensure that the calibration is "good".
        resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
        This is recommended before you run your autonomous, as a bad initial calibration can cause
        an incorrect starting value for x, y, and heading.
         */
        //odo.recalibrateIMU();
        odo.resetPosAndIMU();
    }

    public double headingError(double targetHeading){
        return Heading-targetHeading;
    }

    @Override
    public void execute() {
        executeEX();
    }

    public double X (){
        return X;
    }

    public double Y (){
        return Y;
    }

    public double Heading (){
        return Math.toDegrees(Heading);
    }

    public double getYVelocity(){
        return YVelocity;
    }

    public double getXVelocity(){
        return XVelocity;
    }

    public LambdaCommand update = new LambdaCommand(
            () -> {},
            () -> {

                odo.update();

                XVelocity = odo.getVelX(DistanceUnit.CM);
                YVelocity = odo.getVelY(DistanceUnit.CM);

                Heading = startHeading + odo.getHeading(AngleUnit.DEGREES);

//                if (Math.toDegrees(Heading) < 0){
//                    Heading = Math.toRadians(360 - Math.toDegrees(Heading));
//                } else if (Math.toDegrees(Heading) > 360) {
//                    Heading = Math.toRadians(Math.toDegrees(Heading) - 360);
//                }

                X = startX + odo.getPosX(DistanceUnit.CM);
                Y = startY + odo.getPosY(DistanceUnit.CM);
            },
            () -> false
    );

    public void offsetY(double offset){
        Y += offset;
    }

    public Command resetPosition(double X, double Y, int Heading){
        this.X = startX;
        this.Y = startY;
        startHeading = Heading;
        odo.resetPosAndIMU();
        return resetPosition;
    }

    private final LambdaCommand resetPosition = new LambdaCommand(
            () -> {},
            () -> {

            },
            () -> false
    );


}