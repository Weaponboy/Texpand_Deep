package dev.weaponboy.vision.Testing_SIM;

import static org.opencv.core.Core.inRange;
import static org.opencv.core.CvType.CV_8U;
import static org.opencv.imgproc.Imgproc.CHAIN_APPROX_SIMPLE;
import static org.opencv.imgproc.Imgproc.COLOR_RGB2HSV;
import static org.opencv.imgproc.Imgproc.RETR_TREE;
import static org.opencv.imgproc.Imgproc.boundingRect;
import static org.opencv.imgproc.Imgproc.dilate;
import static org.opencv.imgproc.Imgproc.erode;
import static org.opencv.imgproc.Imgproc.findContours;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class NormalOpenCvPipe extends OpenCvPipeline {

    Mat redMat = new Mat();

    public Scalar redLower = new Scalar(0,30,120);
    public Scalar redHigher = new Scalar(15,255,255);

    ArrayList<MatOfPoint> redContours = new ArrayList<>();
    Mat redHierarchy = new Mat();

    @Override
    public Mat processFrame(Mat input) {

        Imgproc.cvtColor(input, redMat, COLOR_RGB2HSV);

        inRange(redMat, redLower, redHigher, redMat);

        erode(redMat, redMat, new Mat(5, 5, CV_8U));

        dilate(redMat, redMat, new Mat(5, 5, CV_8U));

        findContours(redMat, redContours, redHierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

        ArrayList<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();

        Imgproc.findContours(redMat, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        for (int i = 0; i < contours.size(); i++){
            Rect rect = boundingRect(contours.get(i));
            if (rect.area() > 30000 && rect.area() < 35000){
                redContours.add(contours.get(i));
            }
        }

        ArrayList<Point> cornerPoints = new ArrayList<>();

        for (MatOfPoint contour : redContours) {

            MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
            MatOfPoint2f approxCurve = new MatOfPoint2f();
            double epsilon = 0.04 * Imgproc.arcLength(contour2f, true);
            Imgproc.approxPolyDP(contour2f, approxCurve, epsilon, true);


                for (Point p : approxCurve.toArray()) {
                    cornerPoints.add(p);
                    Imgproc.circle(input, p,4, redHigher);
                }
        }

//        Imgproc.drawContours(input, redContours, -1, new Scalar(0, 255, 0), 2);

        redContours.clear();

        return input;
    }

}
