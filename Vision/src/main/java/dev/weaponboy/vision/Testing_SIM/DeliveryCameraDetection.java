package dev.weaponboy.vision.Testing_SIM;

import static org.opencv.core.Core.inRange;
import static org.opencv.core.Core.sort;
import static org.opencv.core.CvType.CV_8U;
import static org.opencv.imgproc.Imgproc.COLOR_RGB2HSV;
import static org.opencv.imgproc.Imgproc.boundingRect;
import static org.opencv.imgproc.Imgproc.dilate;
import static org.opencv.imgproc.Imgproc.erode;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;


import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

public class DeliveryCameraDetection implements VisionProcessor{

    Mat redMat = new Mat();

    public Scalar blueLower = new Scalar(103.4, 114, 52);
    public Scalar blueHigher = new Scalar(119, 255, 255);

    public Scalar yellowLower = new Scalar(0, 80.8, 126.1);
    public Scalar yellowHigher = new Scalar(26, 255, 255);

    public Scalar redLower = new Scalar(9, 92, 104);
    public Scalar redHigher = new Scalar(38,255,255);

    ArrayList<MatOfPoint> redContoursSingle = new ArrayList<>();
    ArrayList<Rect> redRectsSingle = new ArrayList<>();

    ArrayList<MatOfPoint> redContoursMulti = new ArrayList<>();
    ArrayList<Rect> redRectsMulti = new ArrayList<>();

    Size rectSize = new Size(280, 90);

    Point center = new Point(200, 200);

    Point rectCenter = new Point(200, 200);
    Point contourCenter = new Point(200, 200);

    double angle = 30.0;

    Paint red = new Paint();

    ArrayList<Point> topY = new ArrayList<>();

    RotatedRect rotatedRect = new RotatedRect(center, rectSize, angle);

    double maxShort = 180;
    double minShort = 60;

    double minLong = 180;
    double maxLong = 350;

    double singleMinRatio = 2.2;
    double singleMaxRatio = 2.6;

    double railTarget;

    public boolean isRotate() {
        return rotate;
    }

    boolean rotate = false;

    public double getSlidesDelta() {
        return slidesDelta;
    }

    public double getRailTarget() {
        return railTarget;
    }

    double slidesDelta;

    public int getWidth() {
        return width;
    }

    public int getHeight() {
        return height;
    }

    int width = 1280;
    int height = 960;

    double pixelsToCmWidth = (double) getHeight() / 28;
    double pixelsToCmHeight = (double) getWidth() / 46;

    Point centerDet = new Point();

    int avCounter = 0;
    ArrayList<Point> pointsAve = new ArrayList<>();

    public double getAngleRotate() {
        return angleRotate;
    }

    double angleRotate;
    double ratio;

    public void resetAve(){
        avCounter = 0;
        pointsAve.clear();
    }

    Rect ROI = new Rect(250, 0, 620, 960);

    Point TargetPoint;

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos){

        Mat frameSub = frame.submat(ROI);

        Imgproc.cvtColor(frameSub, redMat, COLOR_RGB2HSV);

//        redMat = redMat.submat(ROI);

        inRange(redMat, redLower, redHigher, redMat);

        erode(redMat, redMat, new Mat(5, 5, CV_8U));

        dilate(redMat, redMat, new Mat(5, 5, CV_8U));

        ArrayList<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();

        Imgproc.findContours(redMat, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);


        for (int i = 0; i < contours.size(); i++){
            Rect rect = boundingRect(contours.get(i));
            if (rect.area() > 15000 && rect.area() < 120000){
                redContoursSingle.add(contours.get(i));
                redRectsSingle.add(rect);
            }
        }

        ArrayList<Point> centerPoints = new ArrayList<>();

        for (int i = 0; i < redContoursSingle.size(); i++){
            centerPoints.add(getContourCenter(redContoursSingle.get(i)));
        }

        ArrayList<MatOfPoint> sortedContours = sortContoursByY(redContoursSingle, centerPoints);

////        for (int i = 0; i < sortedContours.size(); i++) {
////
//            Point center = getContourCenter(sortedContours.get(i));
////            Imgproc.circle(frameSub, center, 5, new Scalar(255, 0, 0), -1);
////            Imgproc.putText(frameSub, String.valueOf(i+1), center, 2, 1, new Scalar(255, 0, 0), 4, 2, false);
////
////        }

        Point center = getContourCenter(sortedContours.get(sortedContours.size()-1));
        Imgproc.circle(frameSub, center, 5, new Scalar(0, 0, 255), -1);
        Imgproc.putText(frameSub, String.valueOf(sortedContours.size()), center, 2, 1, new Scalar(0, 0, 255), 4, 2, false);

        if (!sortedContours.isEmpty()){

            MatOfPoint2f contour2f = new MatOfPoint2f(sortedContours.get(sortedContours.size()-1).toArray());

            RotatedRect rotatedRect = Imgproc.minAreaRect(contour2f);

            Rect closestRest = boundingRect(sortedContours.get(sortedContours.size()-1));
            MatOfPoint closestContour = sortedContours.get(sortedContours.size()-1);

            boolean isWidthLonger = rectSize.width > rectSize.height;
            angle = rotatedRect.angle;

            // Adjust the angle to ensure it's based on the long side of the rectangle
            if (isWidthLonger) {
                angle = angle + 90; // Add 90 degrees if height is the longer side
            }

            // Normalize the angle:
            // Positive angle = tipped right, Negative angle = tipped left
            if (angle > 90) {
                angle -= 180; // Normalize to the range [-90, 90]
            }

//            if (!isWidthLonger || Math.abs(rectSize.width - rectSize.height) < 20) {
//                angle = Math.abs(angle - 90); // Add 90 degrees if height is the longer side
//            }

            if (closestRest.area() > 1500){
                Point lowestPoint = getLowestYPoint(closestContour);
                Point contourCenter = getContourCenter(closestContour);

                double relativeXCorrective;
                double relativeYCorrective;
                Point targetPoint;

                assert lowestPoint != null;

                relativeXCorrective = (26) * Math.sin(Math.toRadians(angle)) + (26) * Math.cos(Math.toRadians(angle));
                relativeYCorrective = (26) * Math.cos(Math.toRadians(angle)) - (26) * Math.sin(Math.toRadians(angle));

                if (contourCenter.x > lowestPoint.x){
                    targetPoint = new Point(lowestPoint.x + relativeXCorrective, lowestPoint.y + relativeYCorrective);
                } else {
                    targetPoint = new Point(lowestPoint.x - relativeXCorrective, lowestPoint.y + relativeYCorrective);
                }

                Imgproc.circle(frameSub, targetPoint, 5, new Scalar(0, 0, 255), -1);
                Imgproc.circle(frameSub, lowestPoint, 5, new Scalar(0, 0, 255), -1);
//
                Imgproc.putText(frameSub, String.valueOf(angle), new Point(20, 100), 2, 1, new Scalar(0, 0, 255), 4, 2, false);
//                Imgproc.putText(frameSub, String.valueOf(relativeXCorrective), new Point(20, 200), 2, 1, new Scalar(0, 0, 255), 4, 2, false);
//                Imgproc.putText(frameSub, String.valueOf(Math.hypot(34,14)), new Point(20, 300), 2, 1, new Scalar(0, 0, 255), 4, 2, false);


            }else {

            }


            double ratio = (double) closestRest.width / closestRest.height;
            Imgproc.putText(frameSub, String.valueOf(ratio), new Point(20, 600), 2, 1, new Scalar(0, 0, 255), 4, 2, false);
            Imgproc.rectangle(frameSub, closestRest, new Scalar(0, 0, 255));

        }

        for (MatOfPoint contour : sortedContours) {
            Imgproc.drawContours(frameSub, Arrays.asList(contour), -1, new Scalar(0, 255, 0), 2);
        }


        //rect.area() > 5000 && rect.area() < 15000
        //Single rect size


//        if (!redContoursSingle.isEmpty()){
//
//            rotate = true;
//
//            ArrayList<Point> centerPoints = new ArrayList<>();
//
//            for (MatOfPoint contour: redContoursSingle){
//                Moments moments = Imgproc.moments(contour);
//
//                double cx = moments.get_m10() / moments.get_m00();
//                double cy = moments.get_m01() / moments.get_m00();
//
//                centerPoints.add(new Point(cx, cy));
//            }
//
//            int closestIndex = findClosestPoint(centerPoints, new Point(640, 480));
//
//            Moments moments = Imgproc.moments(redContoursSingle.get(closestIndex));
//
//            double cx = moments.get_m10() / moments.get_m00();
//            double cy = moments.get_m01() / moments.get_m00();
//
//            avCounter++;
//            pointsAve.add(new Point(cx, cy));
//            MatOfPoint2f contour2f = new MatOfPoint2f(redContoursSingle.get(closestIndex).toArray());
//
//            RotatedRect rotatedRect = Imgproc.minAreaRect(contour2f);
//
//            double angle = rotatedRect.angle;
//            Size rectSize = rotatedRect.size;
//
//            // Determine the long side of the rectangle
//            boolean isWidthLonger = rectSize.width > rectSize.height;
//
//            if (isWidthLonger) {
//                ratio = rotatedRect.size.width/rotatedRect.size.height;
//            }else {
//                angle = angle + 90;
//                ratio = rotatedRect.size.height/rotatedRect.size.width;
//            }
//
//            if (angle > 90) {
//                angle -= 180; // Normalize to the range [-90, 90]
//            }
//
//            angleRotate = -angle;
//
//            center = new Point(cx, cy);
//
//            Point[] rectPoints = new Point[4];
//            rotatedRect.points(rectPoints);  // Fills the array with the corner points
//
//            // Convert the points to a list for drawing the contour
//            MatOfPoint points = new MatOfPoint(rectPoints);
//
//            // Draw the rotated rectangle using drawContours
//            Imgproc.drawContours(frame,
//                    java.util.Collections.singletonList(points),
//                    -1,   // Index of contour (only one here)
//                    new Scalar(255, 0, 0),  // Color in BGR format (blue)
//                    2);
//
//            avCounter++;
//            pointsAve.add(center);
//
//        } else if (!redContoursMulti.isEmpty() && redContoursSingle.isEmpty()) {
//
//            rotate = true;
//
//            for (int i = 0; i < redContoursMulti.size(); i++) {
//
//                Moments moments = Imgproc.moments(redContoursMulti.get(i));
//
//                double cx = moments.get_m10() / moments.get_m00();
//                double cy = moments.get_m01() / moments.get_m00();
//
//                contourCenter = new Point(cx, cy);
//                MatOfPoint2f contour2f = new MatOfPoint2f(redContoursMulti.get(i).toArray());
//
//                RotatedRect rotatedRect = Imgproc.minAreaRect(contour2f);
//
//                rectCenter = rotatedRect.center;
//
//                double angle = rotatedRect.angle;
//                Size rectSize = rotatedRect.size;
//
//                // Determine the long side of the rectangle
//                boolean isWidthLonger = rectSize.width > rectSize.height;
//
//                // Adjust the angle to ensure it's based on the long side of the rectangle
//                if (isWidthLonger) {
//                    angle = angle + 90; // Add 90 degrees if height is the longer side
//                }
//
//                // Normalize the angle:
//                // Positive angle = tipped right, Negative angle = tipped left
//                if (angle > 90) {
//                    angle -= 180; // Normalize to the range [-90, 90]
//                }
//
//                angleRotate = angle;
//
//                Point[] rectPoints = new Point[4];
//                Point[] rectPointsDraw = new Point[4];
//                rotatedRect.points(rectPoints);
//                rotatedRect.points(rectPointsDraw);  // Fills the array with the corner points// Fills the array with the corner points
//
//                Point p1 = rectPoints[0];      // First point of the side
//                Point p2 = rectPoints[(1) % 4];  // Second point of the side (next in the loop)
//
//                if (!isWidthLonger){
//                    p1 = rectPoints[0];
//                    p2 = rectPoints[(1) % 4];
//                }else {
//                    p1 = rectPoints[1];
//                    p2 = rectPoints[(2) % 4];
//                }
//
//                double deltaY = p2.y - p1.y;
//                double deltaX = p2.x - p1.x;
//
//                if (deltaX == 0) {
//                    return Double.POSITIVE_INFINITY;  // Vertical line has an infinite slope
//                }
//
//                double slope = deltaY / deltaX;
//
////                angleRotate = slope;
//
//                double hypot;
//
//                if (isWidthLonger){
//                    hypot = rectSize.width;
//                }else {
//                    hypot = rectSize.height;
//                }
//
//                if (hypot > 350){
//
//                }else {
//                    if (isWidthLonger){
//                        hypot = rectSize.height;
//                    }else {
//                        hypot = rectSize.width;
//                    }
//                }
//
//                Point deltaPoint = calculateDeltas(slope, (hypot/2)-60);
//
//                boolean isBottom = findIsBottom(rotatedRect, contourCenter);
//
//                Imgproc.putText(frame, String.valueOf(isBottom), new Point(20, 160), 2, 1, new Scalar(0, 255, 0), 4, 2, false);
//
//
//                if (isBottom && slope > 0){
//                    center = new Point(rectCenter.x + deltaPoint.x,  rectCenter.y + deltaPoint.y);
//                } else if (isBottom && slope < 0) {
//                    center = new Point(rectCenter.x - deltaPoint.y,  rectCenter.y + deltaPoint.x);
//                }else if (isBottom) {
//                    center = new Point(rectCenter.x - deltaPoint.x,  rectCenter.y - deltaPoint.y);
//                } else {
//                    center = new Point(rectCenter.x + deltaPoint.x,  rectCenter.y + deltaPoint.y);
//                }
//
////                contourCenter = deltaPoint;
//
//                avCounter++;
//                pointsAve.add(center);
//
//                // Convert the points to a list for drawing the contour
//                MatOfPoint points = new MatOfPoint(rectPointsDraw);
//
//                // Draw the rotated rectangle using drawContours
//                Imgproc.drawContours(frame,
//                        java.util.Collections.singletonList(points),
//                        -1,   // Index of contour (only one here)
//                        new Scalar(255, 255, 0),  // Color in BGR format (blue)
//                        2);
//
//            }
//
//        }

        //22.7 at current point
        //start slides at 0
        //-5 to grip

        contours.clear();
        redContoursSingle.clear();
        redRectsSingle.clear();
        redContoursMulti.clear();
        redRectsMulti.clear();
        topY.clear();
        frameSub.copyTo(frame);

        redMat.release();

        Imgproc.putText(frameSub, String.valueOf(sortedContours.size()), new Point(20, 800), 2, 1, new Scalar(0, 255, 0), 4, 2, false);
//        Imgproc.putText(frame, String.valueOf(slidesDelta), new Point(20, 80), 2, 1, new Scalar(0, 255, 0), 4, 2, false);
//        Imgproc.putText(frame, String.valueOf(angleRotate), new Point(20, 120), 2, 1, new Scalar(0, 255, 0), 4, 2, false);
////        Imgproc.putText(frame, String.valueOf(ratio), new Point(20, 160), 2, 1, new Scalar(0, 255, 0), 4, 2, false);

        return null;
    }

    public static Point getContourCenter(MatOfPoint contour) {
        Moments moments = org.opencv.imgproc.Imgproc.moments(contour);

        double cx = moments.get_m10() / moments.get_m00();
        double cy = moments.get_m01() / moments.get_m00();

        return new Point(cx, cy); // Returns the center point (cx, cy)
    }

    public static ArrayList<MatOfPoint> sortContoursByY(ArrayList<MatOfPoint> contours, ArrayList<Point> centerPoints) {
        // Create a list of indices for sorting
        List<Integer> indices = new ArrayList<>();
        for (int i = 0; i < centerPoints.size(); i++) {
            indices.add(i);
        }

        // Sort the indices based on the y value of the center points
        Collections.sort(indices, new Comparator<Integer>() {
            @Override
            public int compare(Integer index1, Integer index2) {
                return Double.compare(centerPoints.get(index1).y, centerPoints.get(index2).y);
            }
        });

        // Create a new array of sorted contours
        ArrayList<MatOfPoint> sortedContours = new ArrayList<>();
        for (int i = 0; i < indices.size(); i++) {
            sortedContours.add(contours.get(indices.get(i)));
        }

        return sortedContours;
    }

    public static Point getLowestYPoint(MatOfPoint contour) {
        List<Point> points = contour.toList(); // Convert the MatOfPoint to a list of Points

        if (points.isEmpty()) {
            return null;  // Return null if the contour is empty
        }

        // Initialize with the first point
        Point lowestYPoint = points.get(0);

        // Loop through the rest of the points to find the one with the lowest Y value
        for (Point point : points) {
            if (point.y < lowestYPoint.y) {  // Note: higher Y value means it's lower on the screen
                lowestYPoint = point;
            }
        }

        return lowestYPoint;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        red.setColor(Color.BLUE);
        red.setStyle(Paint.Style.STROKE);
        red.setStrokeWidth(scaleCanvasDensity * 4);

        if (avCounter >= 1){
            Point center = findAve(pointsAve, avCounter);
            canvas.drawCircle((float) center.x*scaleBmpPxToCanvasPx, (float) center.y*scaleBmpPxToCanvasPx, 4, red);
            avCounter = 0;
            pointsAve.clear();

            double stuff = ((getHeight()-center.y) / pixelsToCmWidth);
            double stuff2 = ((getWidth() -  center.x) / pixelsToCmHeight)-((double) 46 /2) - 8;

            railTarget = stuff-8;
            slidesDelta = stuff2;

        }

    }

    public Point findAve(ArrayList<Point> points, int number){

        double x = 0;
        double y = 0;

        for (Point point : points){
            x += point.x;
            y += point.y;
        }

        return new Point((x/number), (y/number));
    }

}
