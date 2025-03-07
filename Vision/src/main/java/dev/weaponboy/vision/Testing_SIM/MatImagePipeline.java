package dev.weaponboy.vision.Testing_SIM;

import static org.opencv.core.Core.inRange;
import static org.opencv.core.CvType.CV_8U;
import static org.opencv.imgproc.Imgproc.CHAIN_APPROX_SIMPLE;
import static org.opencv.imgproc.Imgproc.COLOR_RGB2BGR;
import static org.opencv.imgproc.Imgproc.COLOR_RGB2HSV;
import static org.opencv.imgproc.Imgproc.boundingRect;
import static org.opencv.imgproc.Imgproc.dilate;
import static org.opencv.imgproc.Imgproc.erode;
import static org.opencv.imgproc.Imgproc.findContours;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;

public class MatImagePipeline extends OpenCvPipeline {

    Mat redMat = new Mat();
    Mat redMat2 = new Mat();

    public Scalar redLower = new Scalar(0, 0, 164);
    public Scalar redHigher = new Scalar(222,160,255);

//    public Scalar redLower = new Scalar(40, 60, 60);
//    public Scalar redHigher = new Scalar(160,255,255);

    public Scalar redLower2 = new Scalar(160, 100, 100);
    public Scalar redHigher2 = new Scalar(220,255,255);

    @Override
    public Mat processFrame(Mat input) {

//        Imgproc.cvtColor(input, redMat2, COLOR_RGB2HSV);
        Imgproc.cvtColor(input, redMat2, COLOR_RGB2HSV);

//        inRange(redMat, redLower, redHigher, redMat);
        inRange(redMat2, redLower, redHigher, redMat2);

//        erode(redMat, redMat, new Mat(5, 5, CV_8U));
        erode(redMat2, redMat2, new Mat(5, 5, CV_8U));

//        dilate(redMat, redMat, new Mat(5, 5, CV_8U));
        dilate(redMat2, redMat2, new Mat(5, 5, CV_8U));

//        Mat output = new Mat();
//        Core.bitwise_or(redMat, redMat2, output);

        return redMat2;
    }

}
