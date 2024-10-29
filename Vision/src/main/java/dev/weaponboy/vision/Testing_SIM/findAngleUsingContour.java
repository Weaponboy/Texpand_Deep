package dev.weaponboy.vision.Testing_SIM;

import static org.opencv.core.Core.inRange;
import static org.opencv.core.Core.mean;
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

import dev.weaponboy.nexus_pathing.PathingUtility.RobotPower;

public class findAngleUsingContour implements VisionProcessor{

    Mat redMat = new Mat();

    public Scalar redLower = new Scalar(9, 92, 104);
    public Scalar redHigher = new Scalar(38,255,255);

    ArrayList<MatOfPoint> redContoursSingle = new ArrayList<>();
    ArrayList<Rect> redRectsSingle = new ArrayList<>();

    ArrayList<MatOfPoint> redContoursMulti = new ArrayList<>();
    ArrayList<Rect> redRectsMulti = new ArrayList<>();

    Rect ROI = new Rect(250, 0, 620, 960);

    Point TargetPoint;
    double angle = 0;

    private final double viewSizeCMXAxis = 76;
    private final double viewSizeCMYAxis = 47;

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos){

        Mat frameSub = frame.submat(ROI);

        Imgproc.cvtColor(frameSub, redMat, COLOR_RGB2HSV);

        inRange(redMat, redLower, redHigher, redMat);

        erode(redMat, redMat, new Mat(5, 5, CV_8U));

        dilate(redMat, redMat, new Mat(5, 5, CV_8U));

        ArrayList<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();

        Imgproc.findContours(redMat, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);

        for (int i = 0; i < contours.size(); i++){
            Rect rect = boundingRect(contours.get(i));
            if (rect.area() > 6000 && rect.area() < 150000){
                redContoursSingle.add(contours.get(i));
                redRectsSingle.add(rect);
            }
        }

        ArrayList<Point> centerPointsSingle = new ArrayList<>();

        for (int i = 0; i < redContoursSingle.size(); i++){
            centerPointsSingle.add(getContourCenter(redContoursSingle.get(i)));
        }

        ArrayList<MatOfPoint> sortedContoursSingle = sortContoursByY(redContoursSingle, centerPointsSingle);

        for (MatOfPoint contour : sortedContoursSingle) {
            Imgproc.drawContours(frameSub, Arrays.asList(contour), -1, new Scalar(0, 255, 0), 2);
        }

//        for(MatOfPoint contour: sortedContoursSingle){
//
//            Rect rect = boundingRect(contour);
//
//            if (rect.area() > 6000 && rect.area() < 15000){
//
//                MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
//
//                RotatedRect rotatedRect = Imgproc.minAreaRect(contour2f);
//
//                boolean isWidthLonger = rotatedRect.size.width > rotatedRect.size.height;
//                angle = rotatedRect.angle;
//
//                if (!isWidthLonger) {
//                    angle = angle + 90;
//                }
//
//                if (angle > 90) {
//                    angle -= 180;
//                }
//
//                Point contourCenter = getContourCenter(contour);
//
//                double angleMultiplier = (0.9)*(Math.abs(angle))/100;
//
//                if (angleMultiplier > 0){
//                    angleMultiplier = 1-angleMultiplier;
//                } else if (angleMultiplier < 0) {
//                    angleMultiplier = -(1-Math.abs(angleMultiplier));
//                }
//
//                double x = calculateAdjustment(contourCenter.y, 496, 0, 0, 26)*angleMultiplier;
//
//                TargetPoint = new Point(contourCenter.x, contourCenter.y - x);
//
//            }else if (rect.area() > 15000 && rect.area() < 150000){
//
//                MatOfPoint closestContour = contour;
//
//                Point lowestPoint = getLowestYPoint(closestContour);
//                Point contourCenter = getContourCenter(closestContour);
//
//                List<Point> contourPoints = closestContour.toList();
//
//                List<Point> angleArray = null;
//
//                if (lowestPoint.x > contourCenter.x){
//                    int index = contourPoints.indexOf(lowestPoint);
//                    Point before;
//
//                    before = contourPoints.get(index+5);
//
//                    if (before.x > lowestPoint.x){
//                        if (index > 42){
//                            angleArray = contourPoints.subList(index , index-40);
//                        }else {
//                            angleArray = contourPoints.subList(contourPoints.size()-40 , contourPoints.size()-1);
//                        }
//
//                    }else {
//                        if (index+42 < contourPoints.size()){
//                            angleArray = contourPoints.subList(index , index+40);
//                        }else {
//                            angleArray = contourPoints.subList(0 , 40);
//                        }
//                    }
//
//                }else if (lowestPoint.x < contourCenter.x){
//                    int index = contourPoints.indexOf(lowestPoint);
//                    Point before, after;
//
//                    if (index > 5){
//                        before = contourPoints.get(index-5);
//                    }else {
//                        before = contourPoints.get(contourPoints.size()-6);
//                    }
//
//                    if (before.x > lowestPoint.x){
//                        if (index > 62){
//                            angleArray = contourPoints.subList(index-20, index-60);
//                        }else {
//                            angleArray =  contourPoints.subList(contourPoints.size()-60 , contourPoints.size()-21);
//                        }
//
//                    }else {
//                        if (index+42 < contourPoints.size()){
//                            angleArray = contourPoints.subList(index , index+40);
//                        }else {
//                            angleArray = contourPoints.subList(0 , 40);
//                        }
//                    }
//                }
//
//                double slope;
//
//                if (angleArray != null){
//                    slope = calculateLineOfBestFit(angleArray);
//                    double angle = Math.toDegrees(Math.atan(slope));
//
////                for (Point point: angleArray){
////                    Imgproc.circle(frameSub, point, 5, new Scalar(0, 0, 255), -1);
////                }
//
//                    Point anyPoint = angleArray.get(17);
//                    double yIntercept = anyPoint.y - (slope * anyPoint.x);
//
//                    Point intercept = findLowestYPointOnLine(slope, yIntercept, contourPoints, 10);
//
//                    double relativeXCorrective;
//                    double relativeYCorrective;
//
//                    double angleRadians = 0;
//                    double xError = 0;
//                    double yError = -30;
//
//                    if (angle < 0){
//                        angle = 135 + (-angle);
//                        angleRadians = Math.toRadians(angle);
//                    }else {
//                        angle = 230 + (-angle);
//                        angleRadians = Math.toRadians(angle);
//                    }
//
//                    angle = Math.toDegrees(Math.atan(slope));
//
//                    this.angle = angle;
//
//                    double angleMultiplier = ((0.9)*(angle)/100);
//                    double angleMultiplierY = (0.9)*(Math.abs(angle))/100;
//
//                    if (angleMultiplier > 0){
//                        angleMultiplier = 1-angleMultiplier;
//                    } else if (angleMultiplier < 0) {
//                        angleMultiplier = -(1-Math.abs(angleMultiplier));
//                    }
//
//                    relativeXCorrective = (yError) * Math.sin(angleRadians) + (xError) * Math.cos(angleRadians);
//                    relativeYCorrective = (yError) * Math.cos(angleRadians) - (xError) * Math.sin(angleRadians);
//
//                    Point centerPoint = new Point(intercept.x + relativeXCorrective, intercept.y + relativeYCorrective);
//
//                    double x = calculateAdjustment(centerPoint.y, 496, 0, 0, 60);
//                    double y = calculateAdjustment(centerPoint.y, 496, 0, 0, 100);
//
//                    if(angle < 60 && angle > -60){
//                        xError = x * angleMultiplier;
//                        yError = y * angleMultiplierY;
//                    }else {
//                        yError = 0;
//                    }
//
//                    TargetPoint = new Point(intercept.x + relativeXCorrective + xError, intercept.y + relativeYCorrective+yError);
//
//                }else {
//                    Imgproc.putText(frameSub, "Bad", new Point(20, 600), 2, 4, new Scalar(0, 255, 0), 4, 2, false);
//                }
//
//            }
//
//            Imgproc.circle(frameSub, TargetPoint, 5, new Scalar(255, 0, 0), -1);
//            Imgproc.putText(frameSub, String.valueOf((int) angle), TargetPoint, 2, 1, new Scalar(0, 255, 0), 4, 2, false);
//
//        }


//        if (sortedContoursSingle.size() > 0){
//
//        } else if (sortedContoursMulti != null) {
//
//        }

        MatOfPoint Contour = sortedContoursSingle.get(sortedContoursSingle.size()-1);
        Rect rect = boundingRect(Contour);

        if (rect.area() > 6000 && rect.area() < 15000){

            MatOfPoint2f contour2f = new MatOfPoint2f(Contour.toArray());

            RotatedRect rotatedRect = Imgproc.minAreaRect(contour2f);

            boolean isWidthLonger = rotatedRect.size.width > rotatedRect.size.height;
            angle = rotatedRect.angle;

            if (!isWidthLonger) {
                angle = angle + 90;
            }

            if (angle > 90) {
                angle -= 180;
            }

            Point contourCenter = getContourCenter(Contour);

            double angleMultiplier = (0.9)*(Math.abs(angle))/100;

            if (angleMultiplier > 0){
                angleMultiplier = 1-angleMultiplier;
            } else if (angleMultiplier < 0) {
                angleMultiplier = -(1-Math.abs(angleMultiplier));
            }

            double x = calculateAdjustment(contourCenter.y, 496, 0, 0, 26)*angleMultiplier;

            TargetPoint = new Point(contourCenter.x, contourCenter.y - x);

        }else if (rect.area() > 15000 && rect.area() < 150000){

            MatOfPoint closestContour = Contour;

            Point lowestPoint = getLowestYPoint(closestContour);
            Point contourCenter = getContourCenter(closestContour);

            List<Point> contourPoints = closestContour.toList();

            List<Point> angleArray = null;

            if (lowestPoint.x > contourCenter.x){
                int index = contourPoints.indexOf(lowestPoint);
                Point before;

                before = contourPoints.get(index+5);

                if (before.x > lowestPoint.x){
                    if (index > 42){
                        angleArray = contourPoints.subList(index , index-40);
                    }else {
                        angleArray = contourPoints.subList(contourPoints.size()-40 , contourPoints.size()-1);
                    }

                }else {
                    if (index+42 < contourPoints.size()){
                        angleArray = contourPoints.subList(index , index+40);
                    }else {
                        angleArray = contourPoints.subList(0 , 40);
                    }
                }

            }else if (lowestPoint.x < contourCenter.x){
                int index = contourPoints.indexOf(lowestPoint);
                Point before, after;

                if (index > 5){
                    before = contourPoints.get(index-5);
                }else {
                    before = contourPoints.get(contourPoints.size()-6);
                }

                if (before.x > lowestPoint.x){
                    if (index > 62){
                        angleArray = contourPoints.subList(index-20, index-60);
                    }else {
                        angleArray =  contourPoints.subList(contourPoints.size()-60 , contourPoints.size()-21);
                    }

                }else {
                    if (index+42 < contourPoints.size()){
                        angleArray = contourPoints.subList(index , index+40);
                    }else {
                        angleArray = contourPoints.subList(0 , 40);
                    }
                }
            }

            double slope;

            if (angleArray != null){
                slope = calculateLineOfBestFit(angleArray);
                double angle = Math.toDegrees(Math.atan(slope));

//                for (Point point: angleArray){
//                    Imgproc.circle(frameSub, point, 5, new Scalar(0, 0, 255), -1);
//                }

                Point anyPoint = angleArray.get(17);
                double yIntercept = anyPoint.y - (slope * anyPoint.x);

                Point intercept = findLowestYPointOnLine(slope, yIntercept, contourPoints, 10);

                double relativeXCorrective;
                double relativeYCorrective;

                double angleRadians = 0;
                double xError = 0;
                double yError = -30;

                if (angle < 0){
                    angle = 135 + (-angle);
                    angleRadians = Math.toRadians(angle);
                }else {
                    angle = 230 + (-angle);
                    angleRadians = Math.toRadians(angle);
                }

                angle = Math.toDegrees(Math.atan(slope));

                this.angle = angle;

                double angleMultiplier = ((0.9)*(angle)/100);
                double angleMultiplierY = (0.9)*(Math.abs(angle))/100;

                if (angleMultiplier > 0){
                    angleMultiplier = 1-angleMultiplier;
                } else if (angleMultiplier < 0) {
                    angleMultiplier = -(1-Math.abs(angleMultiplier));
                }

                relativeXCorrective = (yError) * Math.sin(angleRadians) + (xError) * Math.cos(angleRadians);
                relativeYCorrective = (yError) * Math.cos(angleRadians) - (xError) * Math.sin(angleRadians);

                Point centerPoint = new Point(intercept.x + relativeXCorrective, intercept.y + relativeYCorrective);

                double x = calculateAdjustment(centerPoint.y, 496, 0, 0, 60);
                double y = calculateAdjustment(centerPoint.y, 496, 0, 0, 100);

                if(angle < 60 && angle > -60){
                    xError = x * angleMultiplier;
                    yError = y * angleMultiplierY;
                }else {
                    yError = 0;
                }

                TargetPoint = new Point(intercept.x + relativeXCorrective + xError, intercept.y + relativeYCorrective+yError);

            }else {
                Imgproc.putText(frameSub, "Bad", new Point(20, 600), 2, 4, new Scalar(0, 255, 0), 4, 2, false);
            }

        }

        Imgproc.circle(frameSub, TargetPoint, 5, new Scalar(255, 0, 0), -1);
        Imgproc.putText(frameSub, String.valueOf((int) angle), TargetPoint, 2, 1, new Scalar(0, 255, 0), 4, 2, false);

        contours.clear();
        redContoursSingle.clear();
        redRectsSingle.clear();
        redContoursMulti.clear();
        redRectsMulti.clear();
        redMat.release();
        frameSub.copyTo(frame);

        return null;
    }

    public Point convertToFieldCoor(RobotPower power){
        double pixelsToCMRelX = ROI.height/viewSizeCMXAxis;
        double pixelsToCMRelY = ROI.width/viewSizeCMYAxis;

        double relYPosition = TargetPoint.x*pixelsToCMRelY;
        double relXPosition = (960 - TargetPoint.y)*pixelsToCMRelX;

        double globalX = relXPosition * Math.cos(Math.toRadians(power.getPivot())) - relYPosition * Math.sin(Math.toRadians(power.getPivot()));
        double globalY = relXPosition * Math.sin(Math.toRadians(power.getPivot())) + relYPosition * Math.cos(Math.toRadians(power.getPivot()));

        return new Point(power.getVertical()+globalX, power.getHorizontal()+globalY);
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
            if (centerPoints.get(indices.get(i)).x > 80 && centerPoints.get(indices.get(i)).x < 550){
                sortedContours.add(contours.get(indices.get(i)));
            }
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
