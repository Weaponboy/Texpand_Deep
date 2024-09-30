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
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

public class redSampePipeline extends OpenCvPipeline {

    Mat redMat = new Mat();

    public Scalar redLower = new Scalar(0,110,20);
    public Scalar redHigher = new Scalar(15,255,255);

    ArrayList<MatOfPoint> redContours = new ArrayList<>();
    ArrayList<MatOfPoint> sortedRedContours = new ArrayList<>();
    Mat redHierarchy = new Mat();

    Size rectSize = new Size(280, 90);
    Point center = new Point(200, 200);
    double angle = 30.0;

    Point topY = new Point(0, 800);
    Point rightX = new Point();

    final double maxHypotLength = 440;
    final double minHypotLength = 440;

    RotatedRect rotatedRect = new RotatedRect(center, rectSize, angle);

    @Override
    public Mat processFrame(Mat input) {

        Imgproc.cvtColor(input, redMat, COLOR_RGB2HSV);

        inRange(redMat, redLower, redHigher, redMat);

        erode(redMat, redMat, new Mat(5, 5, CV_8U));

        dilate(redMat, redMat, new Mat(5, 5, CV_8U));

        Mat redHierarchy = new Mat();
        Imgproc.findContours(redMat, redContours, redHierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        for (MatOfPoint contour : redContours) {
            Imgproc.drawContours(input, Arrays.asList(contour), -1, new Scalar(0, 255, 0), 2);
        }

//        findContours(redMat, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
//        findContours(redMat, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

//        for (int i = 0; i < redContours.size(); i++){
//            Rect rect = boundingRect(redContours.get(i));
//            if (rect.area() > 30000 && rect.area() < 35000){
//                sortedRedContours.add(redContours.get(i));
//            }
//        }

        for (MatOfPoint contour : redContours) {

            List<Point> contourPointsY = Arrays.asList(contour.toArray());
            List<Point> contourPointsX = Arrays.asList(contour.toArray());
            ArrayList<Point> farXPoints = new ArrayList<>();
            ArrayList<Point> closeXPoints = new ArrayList<>();

            contourPointsY.sort(Comparator.comparingDouble(p -> p.y));
            Point centerPoint = contourPointsY.subList(0, 1).get(0);

            for (int i = 0; i < contourPointsY.size(); i++){
                if (Math.abs(centerPoint.x - contourPointsY.get(i).x) < 3){
                    closeXPoints.add(contourPointsY.get(i));
                }
            }

            closeXPoints.sort(Comparator.comparingDouble(p -> p.x));
            centerPoint = closeXPoints.subList(0, 1).get(0);

            if(centerPoint.y < topY.y){
                topY = centerPoint;
            }

            Point xRight = Collections.max(contourPointsX, Comparator.comparingDouble(p -> p.x));

            for (int i = 0; i < contourPointsX.size(); i++){
                if (Math.abs(xRight.x - contourPointsX.get(i).x) < 2){
                    farXPoints.add(contourPointsX.get(i));
                }
            }

            farXPoints.sort(Comparator.comparingDouble(p -> p.y));
            centerPoint = farXPoints.subList(0, 1).get(0);

            if(centerPoint.x > rightX.x ){
                rightX = centerPoint;
            }

        }

        double deltaX = Math.abs(topY.x - rightX.x);
        double deltaY = Math.abs(topY.y - rightX.y);
        double hypot = Math.hypot(deltaY, deltaX);

        double angle = 0;

        if (hypot > 200){
            angle = Math.toDegrees(Math.acos(deltaX/hypot));
        } else if (hypot < 200) {
            angle = Math.toDegrees(Math.asin(deltaX/hypot));
        }

        Imgproc.putText(input, String.valueOf(angle), new Point(200, 200), 2, 1, new Scalar(0, 255, 0), 4, 2, false);
       // Imgproc.putText(input, String.valueOf(Math.toDegrees(Math.acos(deltaX/hypot))), new Point(200, 230), 2, 1, new Scalar(0, 255, 0), 4, 2, false);
        //Imgproc.putText(input, String.valueOf(Math.toDegrees(Math.asin(deltaX/hypot))), new Point(200, 260), 2, 1, new Scalar(0, 255, 0), 4, 2, false);
        Imgproc.putText(input, String.valueOf(hypot), new Point(200, 290), 2, 1, new Scalar(0, 255, 0), 4, 2, false);

        rotatedRect.angle = angle;
        rectSize = new Size(hypot, hypot*0.35);
        rotatedRect.size = rectSize;
        rotatedRect.center = createRotatedRectFromTopLeft(topY, rectSize, angle);

        Imgproc.circle(input, topY,4, redHigher);
        Imgproc.circle(input, rightX,4, redHigher);

        Point[] vertices = new Point[4];
        rotatedRect.points(vertices);

        for (int i = 0; i < 4; i++) {
            Imgproc.line(input, vertices[i], vertices[(i + 1) % 4], new Scalar(0, 255, 0), 2);
        }

        redContours.clear();
        topY = new Point(0, 800);
        rightX = new Point();

        return input;
    }

    public Point createRotatedRectFromTopLeft(Point topLeft, Size size, double angle) {
        double radians = Math.toRadians(angle);

        double centerX = topLeft.x + (size.width / 2) * Math.cos(radians) - (size.height / 2) * Math.sin(radians);
        double centerY = topLeft.y + (size.width / 2) * Math.sin(radians) + (size.height / 2) * Math.cos(radians);
        Point center = new Point(centerX, centerY);

        return center;
    }

    public double getHypot(Point one, Point two){
        double deltaX = Math.abs(one.x - two.x);
        double deltaY = Math.abs(one.y - two.y);
        return Math.hypot(deltaY, deltaX);
    }


}
