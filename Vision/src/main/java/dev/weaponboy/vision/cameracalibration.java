package dev.weaponboy.vision;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;

public class cameracalibration {

    public Mat cameraMatrix;
    public Mat distCoeffs;

    // Default constructor with hardcoded values
    public cameracalibration() {
        this.cameraMatrix = new Mat(3, 3, CvType.CV_64F);
        this.cameraMatrix.put(0, 0, 238.29703809);  // fx
        this.cameraMatrix.put(0, 1, 0);
        this.cameraMatrix.put(0, 2, 320.8044176);   // cx
        this.cameraMatrix.put(1, 0, 0);
        this.cameraMatrix.put(1, 1, 238.26964364);  // fy
        this.cameraMatrix.put(1, 2, 226.71491424);  // cy
        this.cameraMatrix.put(2, 0, 0);
        this.cameraMatrix.put(2, 1, 0);
        this.cameraMatrix.put(2, 2, 1);

        this.distCoeffs = new Mat(1, 5, CvType.CV_64F);
        this.distCoeffs.put(0, 0, -0.00612677118);  // k1
        this.distCoeffs.put(0, 1, 0.0443492327);    // k2
        this.distCoeffs.put(0, 2, -0.00085528653);  // p1
        this.distCoeffs.put(0, 3, -0.00006627661);  // p2
        this.distCoeffs.put(0, 4, -0.0414213314);   // k3
    }

    // Constructor with parameters
    public cameracalibration(Mat cameraMatrix, Mat distCoeffs) {
        this.cameraMatrix = cameraMatrix;
        this.distCoeffs = distCoeffs;
    }

    // Method to undistort an image
    public Mat undistortImage(Mat inputImage) {
        Mat undistortedImage = new Mat();
        Calib3d.undistort(inputImage, undistortedImage, cameraMatrix, distCoeffs);
        return undistortedImage;
    }

    // Getters for cameraMatrix and distCoeffs if needed
    public Mat getCameraMatrix() {
        return cameraMatrix;
    }

    public Mat getDistCoeffs() {
        return distCoeffs;
    }
}
