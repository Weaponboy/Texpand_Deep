package dev.weaponboy.vision.Testing_SIM;

import static org.opencv.core.Core.inRange;
import static org.opencv.core.CvType.CV_8U;
import static org.opencv.imgproc.Imgproc.COLOR_RGB2HSV;
import static org.opencv.imgproc.Imgproc.boundingRect;
import static org.opencv.imgproc.Imgproc.dilate;
import static org.opencv.imgproc.Imgproc.erode;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;


public class findAngleUsingContour_SIM implements VisionProcessor{

//    public ArrayList<detectionData> detections = new ArrayList<>();

    Mat redMat = new Mat();

    public Scalar redLower = new Scalar(9, 92, 100);
    public Scalar redHigher = new Scalar(38,255,255);

    ArrayList<MatOfPoint> redContoursSingle = new ArrayList<>();
    ArrayList<Rect> redRectsSingle = new ArrayList<>();

    ArrayList<MatOfPoint> redContoursMulti = new ArrayList<>();
    ArrayList<Rect> redRectsMulti = new ArrayList<>();

    Rect ROI = new Rect(250, 0, 525, 900);

    Point TargetPoint;

    public double getAngle() {
        return angle;
    }

    double angle = 0;

    private final double viewSizeCMXAxis = 70;
    private final double viewSizeCMYAxis = 45;

    public Mat frameSub;

    public boolean isScanning() {
        return scanning;
    }

    public void setScanning(boolean scanning) {
        this.scanning = scanning;
    }

    boolean scanning = true;

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos){

        frameSub = frame.submat(ROI);

        if (scanning){

            Imgproc.cvtColor(frameSub, redMat, COLOR_RGB2HSV);

            inRange(redMat, redLower, redHigher, redMat);

            erode(redMat, redMat, new Mat(5, 5, CV_8U));

            dilate(redMat, redMat, new Mat(5, 5, CV_8U));

            ArrayList<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();

            Imgproc.findContours(redMat, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);

            for (int i = 0; i < contours.size(); i++){
                Rect rect = boundingRect(contours.get(i));
                if (rect.area() > 200 && rect.area() < 150000){
                    redContoursSingle.add(contours.get(i));
                    redRectsSingle.add(rect);
                }
            }

//            ArrayList<Point> centerPointsSingle = new ArrayList<>();
//
//            for (int i = 0; i < redContoursSingle.size(); i++){
//                centerPointsSingle.add(getContourCenter(redContoursSingle.get(i)));
//            }
//
//            ArrayList<MatOfPoint> sortedContoursSingle = sortContoursByYAndCenter(redContoursSingle, centerPointsSingle, ROI.height, ROI.width);

            for (MatOfPoint contour : redContoursSingle) {
                Imgproc.drawContours(frameSub, Arrays.asList(contour), -1, new Scalar(0, 255, 0), 2);
            }

//            MatOfPoint Contour = sortedContoursSingle.get(0);

//            if (!sortedContoursSingle.isEmpty()) {
//
////                MatOfPoint Contour = sortedContoursSingle.get(0);
//                Rect rect = boundingRect(Contour);
//
//                if (rect.area() > 4000 && rect.area() < 15000) {
//
//                    MatOfPoint2f contour2f = new MatOfPoint2f(Contour.toArray());
//
//                    RotatedRect rotatedRect = Imgproc.minAreaRect(contour2f);
//
//                    boolean isWidthLonger = rotatedRect.size.width > rotatedRect.size.height;
//                    angle = rotatedRect.angle;
//
//                    if (!isWidthLonger) {
//                        angle = angle + 90;
//                    }
//
//                    if (angle > 90) {
//                        angle -= 180;
//                    }
//
//                    Point contourCenter = getContourCenter(Contour);
//
//                    double angleMultiplier = (0.9) * (Math.abs(angle)) / 100;
//
//                    if (angleMultiplier > 0) {
//                        angleMultiplier = 1 - angleMultiplier;
//                    } else if (angleMultiplier < 0) {
//                        angleMultiplier = -(1 - Math.abs(angleMultiplier));
//                    }
//
//                    double x = calculateAdjustment(contourCenter.y, 496, 0, 0, 20);
//
//                    TargetPoint = new Point(contourCenter.x, contourCenter.y - x);
//                    Imgproc.circle(frameSub, TargetPoint, 5, new Scalar(255, 0, 0), -1);
//                    Imgproc.putText(frameSub, String.valueOf((int) angle), TargetPoint, 2, 1, new Scalar(0, 255, 0), 4, 2, false);
//
////                        detections.add(new detectionData(System.nanoTime(), TargetPoint, angle));
//
//                } else if (rect.area() > 1000 && rect.area() < 280000) {
//
//                    MatOfPoint closestContour = Contour;
//
//                    Imgproc.drawContours(frameSub, Arrays.asList(closestContour), -1, new Scalar(0, 255, 0), 2);
//
//                    Point lowestPoint = getLowestYHighestXPoint(closestContour);
//                    Point contourCenter = getContourCenter(closestContour);
//
//                    Imgproc.circle(frameSub, lowestPoint, 5, new Scalar(0, 0, 0), -1);
//
//                    List<Point> contourPoints = closestContour.toList();
//
//                    List<Point> angleArray = null;
//
//                    if (lowestPoint.x > contourCenter.x) {
//                        int index = contourPoints.indexOf(lowestPoint);
//                        Point before;
//
//                        before = contourPoints.get(index + 5);
//
//                        if (before.x > lowestPoint.x) {
//                            if (index > 22) {
//                                angleArray = contourPoints.subList(index, index - 20);
//                            } else {
//                                angleArray = contourPoints.subList(contourPoints.size() - 20, contourPoints.size() - 1);
//                            }
//
//                        } else {
//                            if (index + 22 < contourPoints.size()) {
//                                angleArray = contourPoints.subList(index, index + 20);
//                            } else {
//                                angleArray = contourPoints.subList(0, 20);
//                            }
//                        }
//
//                    } else if (lowestPoint.x < contourCenter.x) {
//                        int index = contourPoints.indexOf(lowestPoint);
//                        Point before, after;
//
//                        if (index > 5) {
//                            before = contourPoints.get(index - 5);
//                        } else {
//                            before = contourPoints.get(contourPoints.size() - 6);
//                        }
//
//                        if (before.x > lowestPoint.x) {
//                            if (index < 42) {
//                                angleArray = contourPoints.subList(index + 30, index - 60);
//                            } else {
//                                angleArray = contourPoints.subList(contourPoints.size() - 50, contourPoints.size() - 31);
//                            }
//
//                        } else {
//                            if (index + 22 < contourPoints.size()) {
//                                angleArray = contourPoints.subList(index, index + 20);
//                            } else {
//                                angleArray = contourPoints.subList(0, 20);
//                            }
//                        }
//                    }
//
//                    double slope;
//
//                    if (angleArray != null) {
//                        slope = calculateLineOfBestFit(angleArray);
//                        double angle = Math.toDegrees(Math.atan(slope));
//
//                        for (Point point: angleArray){
//                            Imgproc.circle(frameSub, point, 2, new Scalar(0, 0, 255), -1);
//                        }
//
//                        Point anyPoint = angleArray.get(17);
//                        double yIntercept = anyPoint.y - (slope * anyPoint.x);
//
//                        Point intercept = findLowestYPointOnLine(slope, yIntercept, contourPoints, 10);
//
////                            Imgproc.circle(frameSub, intercept, 5, new Scalar(0, 0, 0), -1);
//
//                        double relativeXCorrective;
//                        double relativeYCorrective;
//
//                        double angleRadians = 0;
//                        double xError = 0;
//                        double yError = -35;
//
//                        if (angle < 0) {
//                            angle = 110 + (-angle);
//                            angleRadians = Math.toRadians(angle);
//                        } else {
//                            angle = 265 + (-angle);
//                            angleRadians = Math.toRadians(angle);
//                        }
//
//                        angle = Math.toDegrees(Math.atan(slope));
//
//                        this.angle = angle;
//
//                        double angleMultiplier = ((0.9) * (angle) / 100);
//                        double angleMultiplierY = (0.9) * (Math.abs(angle)) / 100;
//
//                        if (angleMultiplier > 0) {
//                            angleMultiplier = 1 - angleMultiplier;
//                        } else if (angleMultiplier < 0) {
//                            angleMultiplier = -(1 - Math.abs(angleMultiplier));
//                        }
//
//                        relativeXCorrective = (yError) * Math.sin(angleRadians) + (xError) * Math.cos(angleRadians);
//                        relativeYCorrective = (yError) * Math.cos(angleRadians) - (xError) * Math.sin(angleRadians);
//
//                        Point centerPoint = new Point(intercept.x + relativeXCorrective, intercept.y + relativeYCorrective);
//
//                        double x = calculateAdjustment(centerPoint.y, 496, 0, 0, 20);
//                        double y = calculateAdjustment(centerPoint.y, 496, 0, 0, 300);
//
//                        if (angle < 30 && angle > -30) {
//                            xError = x * angleMultiplier;
//                            yError = y * angleMultiplierY;
//                        } else {
//                            yError = 0;
//                        }
//
//                        TargetPoint = new Point(intercept.x + relativeXCorrective + xError, intercept.y + relativeYCorrective + yError);
//                        Imgproc.circle(frameSub, TargetPoint, 5, new Scalar(255, 0, 0), -1);
//                        Imgproc.putText(frameSub, String.valueOf((int) angle), TargetPoint, 2, 1, new Scalar(0, 0, 0), 4, 2, false);
//
//                    } else {
//                        Imgproc.putText(frameSub, "Bad", new Point(20, 600), 2, 4, new Scalar(0, 255, 0), 4, 2, false);
//                    }
//
//                }
//
//
//            }

        }

        redContoursSingle.clear();
        redRectsSingle.clear();
        redMat.release();
        frameSub.copyTo(frame);

        frameSub.release();

        return null;
    }

    public Point convertToFieldCoor(Point targetPoint){

        double yExtra = calculateAdjustment(TargetPoint.y, 600, 0, 0, 1.75);

        if (TargetPoint.x < 180 || TargetPoint.x > 198){

        }else {
            yExtra = 1;
        }

        double pixelsToCMRelX = viewSizeCMXAxis/ROI.height;
        double pixelsToCMRelY = viewSizeCMYAxis/ROI.width;

        double relYPosition = 0;
        if (TargetPoint.x > 189){
            relYPosition = ((TargetPoint.x-189)*pixelsToCMRelY) * yExtra;
        } else if (TargetPoint.x < 189) {
            relYPosition = ((TargetPoint.x  - 189)*pixelsToCMRelY) * yExtra;
        }
        double relXPosition = (((ROI.height - TargetPoint.y))*pixelsToCMRelX)+16;

        double globalX = relXPosition * Math.cos(Math.toRadians(0)) - relYPosition * Math.sin(Math.toRadians(0));
        double globalY = relXPosition * Math.sin(Math.toRadians(0)) + relYPosition * Math.cos(Math.toRadians(0));

        return new Point(targetPoint.x+globalX, targetPoint.y+globalY);
//        return new Point(pixelsToCMRelX, pixelsToCMRelY);
    }

    public Point getLowestYPoint(MatOfPoint contour) {
        List<Point> points = contour.toList();

        if (points.isEmpty()) {
            return null;
        }

        Point lowestYPoint = points.get(0);

        for (Point point : points) {
            if (point.y < lowestYPoint.y) {
                lowestYPoint = point;
            }
        }

        return lowestYPoint;
    }

    public Point getLowestYHighestXPoint(MatOfPoint contour) {
        List<Point> points = contour.toList();

        if (points.isEmpty()) {
            return null;
        }

        // Find the point with the lowest Y value
        Point lowestYPoint = points.get(0);
        for (Point point : points) {
            if (point.y < lowestYPoint.y) {
                lowestYPoint = point;
            }
        }

        // Define the range for the strip (5 units around the lowest Y point)
        double minY = lowestYPoint.y - 2.5;
        double maxY = lowestYPoint.y + 2.5;

        // Filter points within the strip and find the one with the highest X value
        Point highestXPoint = null;
        for (Point point : points) {
            if (point.y >= minY && point.y <= maxY) {
                if (highestXPoint == null || point.x > highestXPoint.x) {
                    highestXPoint = point;
                }
            }
        }

        return highestXPoint;
    }


    public double calculateAdjustment(double d, double d1, double x1, double d2, double x2) {
        double m = (x2 - x1) / (d2 - d1);

        return m * (d - d1) + x1;
    }

    public Point getContourCenter(MatOfPoint contour) {
        Moments moments = Imgproc.moments(contour);

        double cx = moments.get_m10() / moments.get_m00();
        double cy = moments.get_m01() / moments.get_m00();

        return new Point(cx, cy); // Returns the center point (cx, cy)
    }

    public ArrayList<MatOfPoint> sortContoursByY(ArrayList<MatOfPoint> contours, ArrayList<Point> centerPoints) {
        List<Integer> indices = new ArrayList<>();
        for (int i = 0; i < centerPoints.size(); i++) {
            indices.add(i);
        }

        Collections.sort(indices, new Comparator<Integer>() {
            @Override
            public int compare(Integer index1, Integer index2) {
                return Double.compare(centerPoints.get(index1).y, centerPoints.get(index2).y);
            }
        });

        ArrayList<MatOfPoint> sortedContours = new ArrayList<>();
        for (int i = 0; i < indices.size(); i++) {
            sortedContours.add(contours.get(indices.get(i)));
        }

        return sortedContours;
    }

    public ArrayList<MatOfPoint> sortContoursByYAndCenter(ArrayList<MatOfPoint> contours, ArrayList<Point> centerPoints, double screenHeight, double screenWidth) {
        // Calculate the middle of the screen
        final Point screenCenter = new Point(screenWidth / 2, screenHeight / 2);

        // List of indices to sort
        List<Integer> indices = new ArrayList<>();
        for (int i = 0; i < centerPoints.size(); i++) {
            indices.add(i);
        }

        // Sort by y-coordinate, then by distance to screen center
        Collections.sort(indices, new Comparator<Integer>() {
            @Override
            public int compare(Integer index1, Integer index2) {
                Point point1 = centerPoints.get(index1);
                Point point2 = centerPoints.get(index2);

                // Primary sort by y-coordinate (ascending)
                int yComparison = Double.compare(point1.y, point2.y);
                if (yComparison != 0) {
                    return yComparison;
                }

                // Secondary sort by distance to screen center (ascending)
                double dist1 = Math.hypot(point1.x - screenCenter.x, point1.y - screenCenter.y);
                double dist2 = Math.hypot(point2.x - screenCenter.x, point2.y - screenCenter.y);
                return Double.compare(dist1, dist2);
            }
        });

        // Create sorted contours based on sorted indices
        ArrayList<MatOfPoint> sortedContours = new ArrayList<>();
        for (int index : indices) {
            sortedContours.add(contours.get(index));
        }

        return sortedContours;
    }

    public ArrayList<MatOfPoint> sortContoursByYMulti(ArrayList<MatOfPoint> contours, ArrayList<Point> centerPoints) {
        List<Integer> indices = new ArrayList<>();
        for (int i = 0; i < centerPoints.size(); i++) {
            indices.add(i);
        }

        Collections.sort(indices, new Comparator<Integer>() {
            @Override
            public int compare(Integer index1, Integer index2) {
                return Double.compare(centerPoints.get(index1).y, centerPoints.get(index2).y);
            }
        });

        ArrayList<MatOfPoint> sortedContours = new ArrayList<>();
        for (int i = 0; i < indices.size(); i++) {
            sortedContours.add(contours.get(indices.get(i)));
        }

        return sortedContours;
    }

    public static Point findLowestYPointOnLine(double slope, double yIntercept, List<Point> contourPoints, double tolerance) {
        Point lowestYPoint = null;
        double lowestY = Double.POSITIVE_INFINITY;

        for (Point p : contourPoints) {
            double expectedY = slope * p.x + yIntercept;
            if (Math.abs(p.y - expectedY) <= tolerance) {
                if (p.y < lowestY) {
                    lowestY = p.y;
                    lowestYPoint = p;
                }
            }
        }

        return lowestYPoint;
    }

    private double calculateLineOfBestFit(List<Point> points) {
        int n = points.size();
        double sumX = 0;
        double sumY = 0;
        double sumXY = 0;
        double sumX2 = 0;

        for (Point p : points) {
            double x = p.x;
            double y = p.y;
            sumX += x;
            sumY += y;
            sumXY += x * y;
            sumX2 += x * x;
        }

        double slope = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);
//        double intercept = (sumY - slope * sumX) / n;

        return slope;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

}
