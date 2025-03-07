package dev.weaponboy.command_library.Subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.ArrayList;

import dev.weaponboy.command_library.CommandLibrary.Commands.Command;
import dev.weaponboy.command_library.CommandLibrary.Commands.LambdaCommand;
import dev.weaponboy.command_library.CommandLibrary.OpmodeEX.OpModeEX;
import dev.weaponboy.command_library.CommandLibrary.Subsystem.SubSystem;
import dev.weaponboy.command_library.Hardware.DistanceSensor;
import dev.weaponboy.command_library.Hardware.SensorReadings;

public class OdometryWithKalmanFilter extends SubSystem {

    DistanceSensor backLeft = new DistanceSensor();
    DistanceSensor backRight = new DistanceSensor();
    DistanceSensor right = new DistanceSensor();

    ArrayList<SensorReadings> sensorReadings = new ArrayList<>();

    DcMotorEx leftPod;
    DcMotorEx rightPod;
    DcMotorEx backPod;

    IMU imu; // Added IMU for EKF

    double X, Y, Heading;
    int startHeading;
    public double otherHeading;

    double lastRightPod, lastLeftPod, lastBackPod;
    public double currentRightPod, currentLeftPod, currentBackPod;
    public double rightPodPos, leftPodPos, backPodPos;

    double podTicks = 2000;
    double wheelRadius = 2.4; // cm
    double trackWidth = 24.2; // cm
    double backPodOffset = 9.8; // cm

    double ticksPerCM = ((2.0 * Math.PI) * wheelRadius) / podTicks;
    double cmPerDegreeX = (double) (2) / 360;
    double cmPerDegreeY = ((2.0 * Math.PI) * backPodOffset) / 360;

    double currentXVelocity = 0;
    double currentYVelocity = 0;

    boolean sampleReset = true;

    // EKF Variables
    private double[] state = new double[]{0.0, 0.0, 0.0}; // [x, y, theta] in cm and radians
    private double[][] P = new double[][]{{0.1, 0, 0}, {0, 0.1, 0}, {0, 0, 0.1}}; // Covariance
    private double[][] Q = new double[][]{{0.01, 0, 0}, {0, 0.01, 0}, {0, 0, 0.001}}; // Process noise
    private double[][] R = new double[][]{{0.005}}; // Measurement noise (IMU theta)
    private double lastUpdateTime = 0;

    public boolean isRunningDistanceSensorReset() {
        return runningDistanceSensorReset;
    }

    public void runDistanceSensorReset(boolean sampleReset) {
        runningDistanceSensorReset = true;
        this.sampleReset = sampleReset;
        resetCounter = 0;
        sensorReadings.clear();
    }

    boolean runningDistanceSensorReset = false;
    int resetCounter = 0;

    public OdometryWithKalmanFilter (OpModeEX opModeEX) {
        registerSubsystem(opModeEX, updateEKF);
    }

    public void startPosition(double X, double Y, int Heading) {
        this.X = X;
        this.Y = Y;
        this.startHeading = Heading;
        this.Heading = Math.toRadians(Heading);
        this.state = new double[]{X, Y, Math.toRadians(Heading)}; // Initialize EKF state
    }

    @Override
    public void init() {
        leftPod = getOpModeEX().hardwareMap.get(DcMotorEx.class, "RB");
        rightPod = getOpModeEX().hardwareMap.get(DcMotorEx.class, "RF");
        backPod = getOpModeEX().hardwareMap.get(DcMotorEx.class, "LF");

        backRight.init(getOpModeEX().hardwareMap, "backRight");
        backLeft.init(getOpModeEX().hardwareMap, "backLeft");
        backRight.setOffset(-180);
        backLeft.setOffset(-180);
        right.init(getOpModeEX().hardwareMap, "right");
        right.setOffset(-200);

        // Initialize IMU
        imu = getOpModeEX().hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection = RevHubOrientationOnRobot.LogoFacingDirection.DOWN;
        RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoFacingDirection, usbFacingDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));

        imu.resetYaw();

        lastUpdateTime = getOpModeEX().time;
    }

    public double headingError(double targetHeading) {
        return Heading - targetHeading;
    }

    @Override
    public void execute() {
        executeEX();
        updateVelocity();

        if (runningDistanceSensorReset) {
            resetCounter++;
            sensorReadings.add(new SensorReadings(backRight.getPosition(), backLeft.getPosition(), right.getPosition()));
            if (resetCounter > 20) {
                runningDistanceSensorReset = false;
                SensorReadings averaged;
                double backRight = 0, backLeft = 0, right = 0;
                for (SensorReadings reading : sensorReadings) {
                    backRight += reading.getSen1();
                    backLeft += reading.getSen2();
                    right += reading.getSen3();
                }
                averaged = new SensorReadings((backRight / resetCounter) * 0.1, (backLeft / resetCounter) * 0.1, (right / resetCounter) * 0.1);

                final double distanceFromRobotCenterToSensor = 11;
                final double distanceBetweenSensors = 13.2;
                double readingDifference = averaged.getSen2() - averaged.getSen1();
                double headingError = Math.atan(readingDifference / distanceBetweenSensors);

                double newHeading, newY, newX;
                if (sampleReset) {
                    newHeading = 180 + Math.toDegrees(headingError);
                    newY = 360 - (Math.cos(Math.abs(headingError)) * (averaged.getSen3() + distanceFromRobotCenterToSensor));
                    newX = 360 - (((averaged.getSen1() + averaged.getSen2()) / 2) + 17.5);
                } else {
                    newHeading = 90 + Math.toDegrees(headingError);
                    newX = 360 - (Math.cos(Math.abs(headingError)) * (averaged.getSen3() + distanceFromRobotCenterToSensor));
                    newY = (((averaged.getSen1() + averaged.getSen2()) / 2) + 17.5);
                }
                X = newX;
                Y = newY;
                Heading = Math.toRadians(newHeading);
                state = new double[]{newX, newY, Math.toRadians(newHeading)}; // Reset EKF state
            }
        }
    }

    public double X() {
        return X;
    }

    public double Y() {
        return Y;
    }

    public double Heading() {
        return Math.toDegrees(Heading);
    }

    public double getYVelocity() {
        return currentYVelocity;
    }

    public double getXVelocity() {
        return currentXVelocity;
    }

    public void updateVelocity() {
        double RRXError = ticksPerCM * ((-rightPod.getVelocity() + (-leftPod.getVelocity())) / 2);
        double RRYError = ticksPerCM * -backPod.getVelocity();
        currentXVelocity = RRXError * Math.cos(Heading) - RRYError * Math.sin(Heading);
        currentYVelocity = RRXError * Math.sin(Heading) + RRYError * Math.cos(Heading);
    }

    // EKF Methods
    private double[] predictEKF(double deltaLeft, double deltaRight, double deltaBack, double dt) {
        double[] statePred = new double[3];
        double theta = state[2];
        double dsForward = (deltaLeft + deltaRight) * ticksPerCM / 2;
        double dTheta = (deltaRight - deltaLeft) * ticksPerCM / trackWidth;
        double dsLateral = deltaBack * ticksPerCM;

        statePred[0] = state[0] + dsForward * Math.cos(theta);
        statePred[1] = state[1] + dsForward * Math.sin(theta) + dsLateral;
        statePred[2] = state[2] + dTheta;

        // Jacobian for linearization
        double[][] F = new double[][]{
                {1, 0, -dsForward * Math.sin(theta)},
                {0, 1, dsForward * Math.cos(theta)},
                {0, 0, 1}
        };

        // Update covariance
        double[][] P_pred = matrixAdd(matrixMultiply(matrixMultiply(F, P), transpose(F)), Q);
        P = P_pred;
        return statePred;
    }

    private void updateEKF(double[] statePred, double imuTheta) {
        double[][] H = new double[][]{{0, 0, 1}}; // Measurement model (theta only)
        double z = imuTheta; // IMU measurement
        double y = z - statePred[2]; // Innovation

        double[][] S = matrixAdd(matrixMultiply(matrixMultiply(H, P), transpose(H)), R);
        double[][] K = matrixMultiply(P, transpose(H)); // Simplified Kalman gain (assuming S is scalar)
        K[0][0] /= S[0][0];
        K[1][0] /= S[0][0];
        K[2][0] /= S[0][0];

        // Update state
        state[0] = statePred[0] + K[0][0] * y;
        state[1] = statePred[1] + K[1][0] * y;
        state[2] = statePred[2] + K[2][0] * y;

        // Update covariance
        double[][] I = new double[][]{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
        P = matrixMultiply(matrixSubtract(I, matrixMultiply(K, H)), P);

        // Sync with existing variables
        X = state[0];
        Y = state[1];
        Heading = state[2];
    }

    // Matrix helper methods (since Java lacks NumPy)
    private double[][] matrixMultiply(double[][] A, double[][] B) {
        int m = A.length, n = B[0].length, p = B.length;
        double[][] C = new double[m][n];
        for (int i = 0; i < m; i++) {
            for (int j = 0; j < n; j++) {
                for (int k = 0; k < p; k++) {
                    C[i][j] += A[i][k] * B[k][j];
                }
            }
        }
        return C;
    }

    private double[][] matrixAdd(double[][] A, double[][] B) {
        double[][] C = new double[A.length][A[0].length];
        for (int i = 0; i < A.length; i++) {
            for (int j = 0; j < A[0].length; j++) {
                C[i][j] = A[i][j] + B[i][j];
            }
        }
        return C;
    }

    private double[][] matrixSubtract(double[][] A, double[][] B) {
        double[][] C = new double[A.length][A[0].length];
        for (int i = 0; i < A.length; i++) {
            for (int j = 0; j < A[0].length; j++) {
                C[i][j] = A[i][j] - B[i][j];
            }
        }
        return C;
    }

    private double[][] transpose(double[][] A) {
        double[][] T = new double[A[0].length][A.length];
        for (int i = 0; i < A.length; i++) {
            for (int j = 0; j < A[0].length; j++) {
                T[j][i] = A[i][j];
            }
        }
        return T;
    }

    public LambdaCommand updateLineBased = new LambdaCommand(
            () -> {},
            () -> {
                lastBackPod = currentBackPod;
                lastLeftPod = currentLeftPod;
                lastRightPod = currentRightPod;

                currentBackPod = -backPod.getCurrentPosition();
                currentLeftPod = -leftPod.getCurrentPosition();
                currentRightPod = -rightPod.getCurrentPosition();

                double deltaRight = currentRightPod - lastRightPod;
                double deltaLeft = currentLeftPod - lastLeftPod;
                double deltaBack = currentBackPod - lastBackPod;

                // Line-based update (optional, can switch to EKF)
                double deltaHeading = (ticksPerCM * (deltaRight - deltaLeft)) / (trackWidth + 0.22);
                Heading += deltaHeading;
                if (Math.toDegrees(Heading) < 0) {
                    Heading = Math.toRadians(360 - Math.toDegrees(Heading));
                } else if (Math.toDegrees(Heading) > 360) {
                    Heading = Math.toRadians(Math.toDegrees(Heading) - 360);
                }
                double deltaX = (((deltaRight + deltaLeft) * ticksPerCM) / 2) + (Math.toDegrees(deltaHeading) * cmPerDegreeX);
                double deltaY = (ticksPerCM * deltaBack) - (Math.toDegrees(deltaHeading) * cmPerDegreeY);
                X += deltaX * Math.cos(Heading) - deltaY * Math.sin(Heading);
                Y += deltaX * Math.sin(Heading) + deltaY * Math.cos(Heading);

                // Uncomment to use EKF instead
                // double dt = getOpModeEX().time - lastUpdateTime;
                // lastUpdateTime = getOpModeEX().time;
                // double[] statePred = predictEKF(deltaLeft, deltaRight, deltaBack, dt);
                // Orientation imuOrientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
                // updateEKF(statePred, imuOrientation.firstAngle);
            },
            () -> true
    );

    public LambdaCommand updateEKF = new LambdaCommand(
            () -> {},
            () -> {
                lastBackPod = currentBackPod;
                lastLeftPod = currentLeftPod;
                lastRightPod = currentRightPod;

                currentBackPod = -backPod.getCurrentPosition();
                currentLeftPod = -leftPod.getCurrentPosition();
                currentRightPod = -rightPod.getCurrentPosition();

                double deltaRight = currentRightPod - lastRightPod;
                double deltaLeft = currentLeftPod - lastLeftPod;
                double deltaBack = currentBackPod - lastBackPod;

                double dt = getOpModeEX().time - lastUpdateTime;
                lastUpdateTime = getOpModeEX().time;

                double[] statePred = predictEKF(deltaLeft, deltaRight, deltaBack, dt);
                updateEKF(statePred, imu.getRobotYawPitchRollAngles().getPitch());
            },
            () -> true
    );

    public Command resetPosition(double X, double Y, int Heading) {
        this.X = X;
        this.Y = Y;
        this.startHeading = Heading;
        this.Heading = Math.toRadians(Heading);
        this.state = new double[]{X, Y, Math.toRadians(Heading)};
        return resetPosition;
    }

    private final LambdaCommand resetPosition = new LambdaCommand(
            () -> {},
            () -> {},
            () -> false
    );

    private void updatePodReadings() {
        // Placeholder if needed
    }
}