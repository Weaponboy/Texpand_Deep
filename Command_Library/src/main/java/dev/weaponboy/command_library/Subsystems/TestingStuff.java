package dev.weaponboy.command_library.Subsystems;

import dev.weaponboy.command_library.Hardware.motionProfile;

public class TestingStuff {

    static motionProfile profile = new motionProfile(1700, 210, 75, 1100, 0.35);

    public static void main(String[] args) {
        profile.generateMotionProfile(5, 200);
        int value = 0;
        profile.followProfile(value);
//        while (profile.isSlideRunning()){
//            System.out.println(profile.followProfile(value));
//            value+=3;
//        }

        System.out.println(findCameraViewWidth());

        findCameraScanPosition();

    }

    public static double findCameraViewWidth(){

        //first triangle
        double pivotHeight = 5 + 43;
        double firstHypot = Math.sqrt((pivotHeight*pivotHeight)+(29 * 29));

        double angleA = Math.toDegrees(Math.atan(29/pivotHeight));
        double angleB = Math.toDegrees(Math.atan(pivotHeight/29));
        double angleC = 90;

        //second triangle
        double angleD = 81;
        double angleE = Math.toDegrees(Math.atan(8 / firstHypot));
        double angleF = 180 - angleE - angleD;

        double sideD = firstHypot;
        double sideE = 8;
        double sideF = sideD * (Math.sin(Math.toRadians(angleF)) / Math.sin(Math.toRadians(angleD)));

        //vision triangle
        double angleG = 34.6;
        double angleH = 180 - angleB - angleE;
        double angleI = 180 - angleH - angleG;

        double sideI = sideF;
        double sideG = sideI * (Math.sin(Math.toRadians(angleG)) / Math.sin(Math.toRadians(angleI)));
        double sideH = sideI * (Math.sin(Math.toRadians(angleH)) / Math.sin(Math.toRadians(angleI)));

        //top vision triangle
        double angleJ = angleG;
        double angleK = (180 - angleJ)/2;
        double angleL = (180 - angleJ)/2;

        double sideL = sideI;
        double sideJ = sideL * (Math.sin(Math.toRadians(angleJ)) / Math.sin(Math.toRadians(angleL)));
        double sideK = sideL;

        //bottom vision triangle
        double angleM = angleH - angleK;
        double angleN = 180 - angleL;
        double angleO = angleI;

        double sideN = sideG;
        double sideO = sideJ;
        double sideM = sideN * (Math.sin(Math.toRadians(angleM)) / Math.sin(Math.toRadians(angleN)));

        double targetPoint = sideJ;

        double miniTriangleSide = Math.sqrt(Math.pow(targetPoint, 2) + Math.pow(sideI, 2) - 2 * targetPoint * sideI * Math.cos(Math.toRadians(angleK)));
        double angleMini = Math.toDegrees(targetPoint * Math.sin(Math.toRadians(angleK)) / miniTriangleSide);
        double otherAngle = 180 - angleMini - angleK;

        double invertedOtherAngle = 180 - otherAngle;

        double otherInsideAngle = 180 - invertedOtherAngle - angleM;

        double realWorldPosition = targetPoint * (Math.sin(Math.toRadians(invertedOtherAngle)) / Math.sin(Math.toRadians(otherInsideAngle)));

        System.out.println(sideN);

        return realWorldPosition;
    }

    // Converts pixel position to real-world position
    public static double pixelToRealWorld(double h, double thetaDeg, double fovDeg, double imageHeight, double pixelPosition) {
        // Convert degrees to radians
        double theta = Math.toRadians(thetaDeg);
        double fov = Math.toRadians(fovDeg);

        // Calculate the focal length in pixels
        double f = (imageHeight / 2.0) / Math.tan(fov / 2.0);

        // Calculate the angle alpha for the pixel position (0-300 pixels, centered at 150)
        // Map the pixel to the field of view angle
        double pixelOffset = pixelPosition - 150;  // Center pixel is 150
        double alpha = Math.toRadians((pixelOffset / (imageHeight / 2.0)) * (fovDeg));

        // Add the camera tilt angle to get the effective angle
        double beta = alpha + theta;

        // Calculate the real-world distance (assuming the real-world plane is at height h)
        return h * Math.tan(beta);
    }

    // Wrapper function to compute the real-world position based on user input
    public static double calculateRealWorldDistance(
            double pivotHeight, double angleA, double angleF, double angleG, double pixelPosition
    ) {
        // Calculate the height of the camera from the pivot
        double h = pivotHeight + 8 * Math.cos(Math.toRadians(180 - (angleA + angleF))); // height in meters

        // Camera tilt angle (angleA + angleF), adjusted by -90 for vertical alignment
        double thetaDeg = ((angleA + angleF) - 90); // Camera tilt angle
        if (thetaDeg < 0) {
            thetaDeg = 0; // Ensures the tilt never goes negative (camera facing straight down)
        }

        // Field of view
        double fovDeg = angleG; // Field of view in degrees

        // Image height in pixels (300 pixels)
        double imageHeight = 300; // In pixels

        // Return the real-world distance for the given pixel position
        return pixelToRealWorld(h, thetaDeg, fovDeg, imageHeight, pixelPosition);
    }

    public static double findCameraScanPosition(){

        double pivotHeight = 12 + 43;
        double X = Math.sqrt(Math.pow(pivotHeight, 2)+Math.pow(29, 2));

        double angle = Math.acos(8 * Math.sin(80) / X);

//        System.out.println(Math.toDegrees((Math.asin(29/X))));
//
//        System.out.println(Math.toDegrees((Math.atan(29/pivotHeight))));

        return 188 - ((Math.toDegrees(angle + Math.atan(29/pivotHeight))-90)*0.755);
    }

    public static double scalePosition(double cameraLength, double realLength, double angleDegrees, double cameraPositionFraction) {
        // Convert angle to radians
        double angleRadians = Math.toRadians(angleDegrees);

        // Calculate real-world position
        double realLengthComputed = cameraLength / Math.cos(angleRadians);
        return cameraPositionFraction * realLengthComputed;
    }

    public static double mapCameraToReal(double cameraPosition, double fovDegrees, double distance, double cameraWidth) {
        // Convert FOV to radians
        double fovRadians = Math.toRadians(fovDegrees / 2); // Half FOV

        // Normalize camera position to range [-1, 1]
        double normalizedPosition = (cameraPosition - (cameraWidth / 2)) / (cameraWidth / 2);

        // Compute the angle from the normalized position
        double theta = fovRadians * normalizedPosition;

        // Calculate the real-world position
        return distance * Math.tan(theta);
    }
}
