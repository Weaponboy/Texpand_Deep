package dev.weaponboy.vision.Testing_SIM;

import static org.opencv.core.Core.inRange;
import static org.opencv.core.CvType.CV_8U;
import static org.opencv.imgproc.Imgproc.COLOR_RGB2HSV;
import static org.opencv.imgproc.Imgproc.boundingRect;
import static org.opencv.imgproc.Imgproc.dilate;
import static org.opencv.imgproc.Imgproc.erode;
import static org.opencv.imgproc.Imgproc.findContours;

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
import java.util.Comparator;
import java.util.List;

public class SampleTargeting  implements VisionProcessor {

    Mat redMat = new Mat();

    public Scalar blueLower = new Scalar(103.4, 114, 52);
    public Scalar blueHigher = new Scalar(119, 255, 255);

    public Scalar yellowLower = new Scalar(0, 80.8, 126.1);
    public Scalar yellowHigher = new Scalar(26, 255, 255);

    public Scalar redLower = new Scalar(9, 100, 160);
    public Scalar redHigher = new Scalar(38,255,255);

    ArrayList<MatOfPoint> redContoursSingle = new ArrayList<>();
    ArrayList<Rect> redRectsSingle = new ArrayList<>();

    ArrayList<MatOfPoint> redContoursMulti = new ArrayList<>();
    ArrayList<Rect> redRectsMulti = new ArrayList<>();


    Size rectSize = new Size(280, 90);

    Point center = new Point(200, 200);

    double angle = 30.0;

    Paint red = new Paint();

    ArrayList<Point> topY = new ArrayList<>();
    Point rightX = new Point();

    final double maxHypotLength = 440;
    final double minHypotLength = 440;

    RotatedRect rotatedRect = new RotatedRect(center, rectSize, angle);

    double maxShort = 180;
    double minShort = 60;

    double minLong = 180;
    double maxLong = 350;

    double railTarget;

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

    Rect ROI = new Rect(0, 0, 1000, 960);

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {

        Imgproc.cvtColor(frame, redMat, COLOR_RGB2HSV);

        inRange(redMat, redLower, redHigher, redMat);

        erode(redMat, redMat, new Mat(5, 5, CV_8U));

        dilate(redMat, redMat, new Mat(5, 5, CV_8U));

        ArrayList<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();

        Imgproc.findContours(redMat, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);

        for (int i = 0; i < contours.size(); i++){
            Rect rect = boundingRect(contours.get(i));
            if (rect.area() > 20000 && rect.area() < 42000){
                redContoursSingle.add(contours.get(i));
                redRectsSingle.add(rect);
            }else if (rect.area() > 42000 && rect.area() < 600000){
                redContoursMulti.add(contours.get(i));
                redRectsMulti.add(rect);
            }
        }

        if (!redContoursSingle.isEmpty()){


            Moments moments = Imgproc.moments(redContoursSingle.get(redContoursSingle.size()-1));

            double cx = moments.get_m10() / moments.get_m00();
            double cy = moments.get_m01() / moments.get_m00();

            avCounter++;
            pointsAve.add(new Point(cx, cy));
            MatOfPoint2f contour2f = new MatOfPoint2f(redContoursSingle.get(redContoursSingle.size()-1).toArray());

            RotatedRect rotatedRect = Imgproc.minAreaRect(contour2f);

            double angle = rotatedRect.angle;
            Size rectSize = rotatedRect.size;

            ratio = rotatedRect.size.height/rotatedRect.size.width;
            ratio = rotatedRect.size.height/rotatedRect.size.width;

            // Determine the long side of the rectangle
            boolean isWidthLonger = rectSize.width > rectSize.height;

            // Adjust the angle to ensure it's based on the long side of the rectangle
            if (isWidthLonger) {
                angle = angle + 90; // Add 90 degrees if height is the longer side
            }

            // Normalize the angle:
            // Positive angle = tipped right, Negative angle = tipped left
            if (angle > 90) {
                angle -= 180; // Normalize to the range [-90, 90]
            }

            angleRotate = angle;

            center = new Point(cx, cy);

            double stuff = ((getHeight()-center.y) / pixelsToCmWidth);
            double stuff2 = ((getWidth() -  center.x) / pixelsToCmHeight)-((double) 46 /2) - 5.5;

            railTarget = stuff-4;
            slidesDelta = stuff2;

            Point[] rectPoints = new Point[4];
            rotatedRect.points(rectPoints);  // Fills the array with the corner points

            // Convert the points to a list for drawing the contour
            MatOfPoint points = new MatOfPoint(rectPoints);

            // Draw the rotated rectangle using drawContours
            Imgproc.drawContours(frame,
                    java.util.Collections.singletonList(points),
                    -1,   // Index of contour (only one here)
                    new Scalar(255, 0, 0),  // Color in BGR format (blue)
                    2);

        } else if ((!redContoursMulti.isEmpty())) {

            for (int i = 0; i < redContoursMulti.size(); i++) {

                Moments moments = Imgproc.moments(redContoursMulti.get(i));

                // Calculate centroid
                double cx = moments.get_m10() / moments.get_m00();
                double cy = moments.get_m01() / moments.get_m00();

                avCounter++;
                pointsAve.add(new Point(cx, cy));
                MatOfPoint2f contour2f = new MatOfPoint2f(redContoursMulti.get(i).toArray());

                RotatedRect rotatedRect = Imgproc.minAreaRect(contour2f);

                double angle = rotatedRect.angle;
                Size rectSize = rotatedRect.size;

                // Determine the long side of the rectangle
                boolean isWidthLonger = rectSize.width > rectSize.height;

                // Adjust the angle to ensure it's based on the long side of the rectangle
                if (isWidthLonger) {
                    angle = angle + 90; // Add 90 degrees if height is the longer side
                }

                // Normalize the angle:
                // Positive angle = tipped right, Negative angle = tipped left
                if (angle > 90) {
                    angle -= 180; // Normalize to the range [-90, 90]
                }

                angleRotate = angle;

                center = new Point(cx, cy);

                double stuff = ((getHeight()-center.y) / pixelsToCmWidth);
                double stuff2 = ((getWidth() -  center.x) / pixelsToCmHeight)-((double) 46 /2) - 5.5;

                railTarget = stuff-4;
                slidesDelta = stuff2;

                Point[] rectPoints = new Point[4];
                rotatedRect.points(rectPoints);  // Fills the array with the corner points

                // Convert the points to a list for drawing the contour
                MatOfPoint points = new MatOfPoint(rectPoints);

                // Draw the rotated rectangle using drawContours
                Imgproc.drawContours(frame,
                        java.util.Collections.singletonList(points),
                        -1,   // Index of contour (only one here)
                        new Scalar(255, 255, 0),  // Color in BGR format (blue)
                        2);

            }

        }

        //22.7 at current point
        //start slides at 0
        //-5 to grip

        contours.clear();
        redContoursSingle.clear();
        redRectsSingle.clear();
        redContoursMulti.clear();
        redRectsMulti.clear();
        topY.clear();
//        redMat.copyTo(frame);

        redMat.release();

        Imgproc.putText(frame, String.valueOf(railTarget), new Point(20, 40), 2, 1, new Scalar(0, 255, 0), 4, 2, false);
        Imgproc.putText(frame, String.valueOf(slidesDelta), new Point(20, 80), 2, 1, new Scalar(0, 255, 0), 4, 2, false);
        Imgproc.putText(frame, String.valueOf(angleRotate), new Point(20, 120), 2, 1, new Scalar(0, 255, 0), 4, 2, false);
        Imgproc.putText(frame, String.valueOf(ratio), new Point(20, 160), 2, 1, new Scalar(0, 255, 0), 4, 2, false);

        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        red.setColor(Color.BLUE);
        red.setStyle(Paint.Style.STROKE);
        red.setStrokeWidth(scaleCanvasDensity * 4);

//        canvas.drawCircle((float) centerDet.x, (float) center.y, 4, red);

        canvas.drawCircle((float) center.x*scaleBmpPxToCanvasPx, (float) center.y*scaleBmpPxToCanvasPx, 4, red);


        if (avCounter > 5){
            Point center = findAve(pointsAve, avCounter);
            canvas.drawCircle((float) center.x*scaleBmpPxToCanvasPx, (float) center.y*scaleBmpPxToCanvasPx, 4, red);
            avCounter = 0;
            pointsAve.clear();

            double stuff = ((getHeight()-center.y) / pixelsToCmWidth);
            double stuff2 = ((getWidth() -  center.x) / pixelsToCmHeight)-((double) 46 /2) - 5.5;

            railTarget = stuff-4;
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

    public Point findTopPosition(Mat input, MatOfPoint Contour){
        Point CenterPoint = null;

        List<Point> contourPointsSorted = Arrays.asList(Contour.toArray());
        List<Point> contourPointsSorted2 = Arrays.asList(Contour.toArray());
        ArrayList<Point> farXPoints = new ArrayList<>();
        List<Point> contourPoints = Arrays.asList(Contour.toArray());

        contourPointsSorted.sort((p1, p2) -> Double.compare(p2.x, p1.x));
        Point topPointFirst = contourPointsSorted.get(0);

        Imgproc.putText(input, String.valueOf(topPointFirst.x), new Point(20, 40), 2, 1, new Scalar(0, 255, 0), 4, 2, false);

        if(topPointFirst.x < 800){
            contourPointsSorted.sort(Comparator.comparingDouble(p -> p.x));
            topPointFirst = contourPointsSorted.get(0);
        }

        for (int i = 0; i < contourPointsSorted2.size(); i++){
            if (Math.abs(topPointFirst.x - contourPointsSorted2.get(i).x) < 5){
                farXPoints.add(contourPointsSorted2.get(i));
            }
        }

        Point topPoint;

        if (farXPoints.size() > 60){
            farXPoints.sort(Comparator.comparingDouble(p -> p.y));
            topPoint = farXPoints.subList(0, 1).get(0);
        }else {
            topPoint = topPointFirst;
        }

        int centerIndex = contourPoints.indexOf(topPoint);

        Imgproc.circle(input, topPoint, 3, new Scalar(255,0, 0), -1);
//        Imgproc.putText(input, String.valueOf(centerIndex), new Point(200, 240), 2, 1, new Scalar(0, 255, 0), 4, 2, false);

        List<Point> sublistBefore;
        List<Point> sublistAfter;

        if(centerIndex == 0){
            sublistBefore = new ArrayList<>(contourPoints.subList(contourPoints.size() - 61, contourPoints.size()-21));
            sublistAfter = new ArrayList<>(contourPoints.subList(20, 60));
//
//            for (Point contour : sublistAfter) {
//                Imgproc.circle(input, contour, 4, new Scalar(0, 255, 0), -1);
//            }
//
//            for (Point contour : sublistBefore) {
//                Imgproc.circle(input, contour, 4, new Scalar(0, 255, 255), -1);
//            }

        }else {

            if (centerIndex >= 66 && centerIndex+65 < contourPoints.size()-1){
                sublistAfter = new ArrayList<>(contourPoints.subList(centerIndex + 10, centerIndex + 65));
                sublistBefore = new ArrayList<>(contourPoints.subList(centerIndex - 65, centerIndex - 10));
            }else{
                sublistAfter = new ArrayList<>(contourPoints.subList(10, 65));
                sublistBefore = new ArrayList<>(contourPoints.subList(contourPoints.size() - 66, contourPoints.size()-11));
            }

            for (Point contour : sublistAfter) {
                Imgproc.circle(input, contour, 4, new Scalar(0, 255, 0), -1);
            }

            for (Point contour : sublistBefore) {
                Imgproc.circle(input, contour, 4, new Scalar(0, 255, 255), -1);
            }

        }

        Imgproc.circle(input, topPoint, 4, new Scalar(255, 255, 0), -1);

        double[] line1 = calculateLineOfBestFit(sublistBefore);
        double slope1 = line1[0];
        double intercept1 = line1[1];

        double[] line2 = calculateLineOfBestFit(sublistAfter);
        double slope2 = line2[0];
        double intercept2 = line2[1];

        Point intersection = findIntersection(slope1, intercept1, slope2, intercept2);

//        drawLineSegment(input, slope1, intercept1, new Scalar(255, 0, 0));
//        drawLineSegment(input, slope2, intercept2,  new Scalar(0, 255, 255));
//        Imgproc.circle(input, intersection, 4, new Scalar(0, 0, 255), -1);
//
        Point furthestPoint = null;
        Point furthestPoint2 = null;

        if (!(intersection == null)){
            furthestPoint = findFurthestPointAlongSlope(contourPoints, intersection, slope1, calculateDistanceTolerance(15));
            furthestPoint2 = findFurthestPointAlongSlope(contourPoints, intersection, slope2, calculateDistanceTolerance(15));
        }

        double deltaXFirst = 0;
        double deltaYFirst = 0;
        double firstLength = 0;

        if (!(furthestPoint == null) && !(intersection == null)){
            deltaXFirst = Math.abs(furthestPoint.x - intersection.x);
            deltaYFirst = Math.abs(furthestPoint.y - intersection.y);
            firstLength = Math.hypot(deltaXFirst, deltaYFirst);
            Imgproc.circle(input, furthestPoint, 6, new Scalar(0, 255, 255), -1);
//            Imgproc.putText(input, String.valueOf(deltaXFirst), new Point(200, 200), 2, 1, new Scalar(0, 255, 0), 4, 2, false);
        }

        double deltaXSecond = 0;
        double deltaYSecond = 0;
        double secondLength = 0;

        if (!(furthestPoint2 == null) && !(intersection == null)){
//            Imgproc.circle(input, furthestPoint2, 4, new Scalar(0, 0, 255), -1);
            deltaXSecond = Math.abs(furthestPoint2.x - intersection.x);
            deltaYSecond = Math.abs(furthestPoint2.y - intersection.y);
            secondLength = Math.hypot(deltaYSecond, deltaXSecond);
            Imgproc.circle(input, furthestPoint2, 6, new Scalar(0, 255, 0), -1);
//            Imgproc.putText(input, String.valueOf(deltaXSecond), new Point(200, 240), 2, 1, new Scalar(0, 255, 0), 4, 2, false);
        }


//
//        if (!(intersection == null)){
//
////            if ((secondLength < maxShort && secondLength > minShort) && !(firstLength < maxLong && firstLength > minLong)){
////                firstLength = secondLength*2.5;
////            }else if (!(secondLength < maxShort && secondLength > minShort) && (firstLength < maxLong && firstLength > minLong)){
////                secondLength = firstLength*0.4;
////            }
////
////            if((secondLength < maxLong && secondLength > minLong) && (firstLength < maxLong && firstLength > minLong)){
////
////                if (secondLength > firstLength){
////                    double newFirstLength = firstLength*0.5;
////                    double ratio = firstLength/newFirstLength;
////                    deltaXFirst = deltaXFirst/ratio;
////                    deltaYFirst = deltaYFirst/ratio;
////                    firstLength = newFirstLength;
////                }else{
////                    double newFirstLength = secondLength*0.5;
////                    double ratio = firstLength/newFirstLength;
////                    deltaXSecond = deltaXSecond/ratio;
////                    deltaYSecond = deltaYSecond/ratio;
////                    secondLength = newFirstLength;
////                }
////
////            }
////
////
////            if ((secondLength < maxShort && secondLength > minShort) && !(firstLength < 260 && firstLength > minLong)){
////                double newFirstLength = secondLength*3;
////                double ratio = firstLength/newFirstLength;
////                deltaXFirst = deltaXFirst/ratio;
////                deltaYFirst = deltaYFirst/ratio;
////                firstLength = newFirstLength;
////            } else if (!(secondLength < maxShort && secondLength > minShort) && (firstLength < 260 && firstLength > minLong)){
////                double newSecondLength = firstLength*0.33;
////                double ratio = secondLength/newSecondLength;
////                deltaXSecond = deltaXSecond/ratio;
////                deltaYSecond = deltaYSecond/ratio;
////                secondLength = newSecondLength;
////            }
//
//////
////            else if (!(firstLength < maxShort && firstLength > minShort) && (secondLength < maxLong && secondLength > minLong)){
////                double newFirstLength = secondLength*0.3;
////                double ratio = newFirstLength/firstLength;
////                deltaXFirst = deltaXFirst*ratio;
////                deltaYFirst = deltaYFirst*ratio;
////                firstLength = newFirstLength;
////            }else if ((secondLength < maxShort && secondLength > minShort) && !(firstLength < maxLong && firstLength > minLong)){
////                double newSecondLength = firstLength*0.4;
////                double ratio = newSecondLength/firstLength;
////                deltaXSecond = deltaXSecond*ratio;
////                deltaYSecond = deltaYSecond*ratio;
////                firstLength = newSecondLength;
////            }else if (!(secondLength < maxShort && secondLength > minShort) && (firstLength < maxLong && firstLength > minLong)){
////                double newFirstLength = secondLength*3;
////                double ratio = newFirstLength/secondLength;
////                deltaXFirst = deltaXFirst*ratio;
////                deltaYFirst = deltaYFirst*ratio;
////                secondLength = newFirstLength;
////            }
//
//        }
//
////        if ((secondLength < maxShort && secondLength > minShort) && !(firstLength < maxLong && firstLength > minLong) &&!(intersection == null)){
////            firstLength = secondLength*2.5;
////        }else if (!(secondLength < maxShort && secondLength > minShort) && (firstLength < maxLong && firstLength > minLong) &&!(intersection == null)){
////            secondLength = firstLength*0.4;
////        }
////
////        else if ((firstLength < maxShort && firstLength > minShort) && !(secondLength < maxLong && secondLength > minLong) && !(intersection == null)){
////            secondLength = firstLength*2.5;
////        }else if (!(firstLength < maxShort && firstLength > minShort) && (secondLength < maxLong && secondLength > minLong) && !(intersection == null)){
////            firstLength = secondLength*0.4;
////        }
//
        if (secondLength < firstLength){
            angleRotate = Math.toDegrees(Math.atan(slope2));
        }else {
            angleRotate = Math.toDegrees(Math.atan(slope1));
        }
//
        if ((secondLength < maxShort && secondLength > minShort) && (firstLength < maxLong && firstLength > minLong) &&!(intersection == null)){
            CenterPoint = new Point(intersection.x + (deltaXSecond/2) + (deltaXFirst/2), intersection.y -  (deltaYFirst/2) + (deltaYSecond/2));
//            Imgproc.putText(input, String.valueOf(firstLength), new Point(20, 40), 2, 1, new Scalar(0, 255, 0), 4, 2, false);
//            Imgproc.putText(input, String.valueOf(secondLength), new Point(20, 80), 2, 1, new Scalar(0, 255, 0), 4, 2, false);
        }else if ((firstLength < maxShort && firstLength > minShort) && (secondLength < maxLong && secondLength > minLong) && !(intersection == null)){
            CenterPoint = new Point(intersection.x + (deltaXSecond/2) + (deltaXFirst/2), intersection.y -  (deltaYFirst/2) + (deltaYSecond/2));
//            Imgproc.putText(input, String.valueOf(firstLength), new Point(20, 40), 2, 1, new Scalar(0, 255, 0), 4, 2, false);
//            Imgproc.putText(input, String.valueOf(secondLength), new Point(20, 80), 2, 1, new Scalar(0, 255, 0), 4, 2, false);
        }
//
//        else {
//            Imgproc.putText(input, String.valueOf(firstLength), new Point(20, 40), 2, 1, new Scalar(0, 255, 0), 4, 2, false);
//            Imgproc.putText(input, String.valueOf(secondLength), new Point(20, 80), 2, 1, new Scalar(0, 255, 0), 4, 2, false);
//
//        }

//        Imgproc.putText(input, String.valueOf(firstLength/secondLength), new Point(20, 40), 2, 1, new Scalar(0, 255, 0), 4, 2, false);
//        Imgproc.putText(input, String.valueOf(angleRotate), new Point(20, 80), 2, 1, new Scalar(0, 255, 0), 4, 2, false);

//
//        Imgproc.putText(input, String.valueOf(firstLength), new Point(200, 280), 2, 1, new Scalar(0, 255, 0), 4, 2, false);
//        Imgproc.putText(input, String.valueOf(secondLength), new Point(200, 340), 2, 1, new Scalar(0, 255, 0), 4, 2, false);

//        Imgproc.putText(input, String.valueOf(deltaXSecond), new Point(200, 280), 2, 1, new Scalar(0, 255, 0), 4, 2, false);
//        Imgproc.putText(input, String.valueOf(deltaYSecond), new Point(200, 340), 2, 1, new Scalar(0, 255, 0), 4, 2, false);

        return CenterPoint;
    }

    public List<Point> filterPoints(List<Point> points, double threshold) {
        List<Point> filteredPoints = new ArrayList<>();

        if (points.isEmpty()) return filteredPoints;

        Point firstPoint = points.get(0);

        for (Point point : points) {
            double distance = Math.hypot(Math.abs(firstPoint.x- point.x), Math.abs(firstPoint.y- point.y));

            if (distance <= threshold) {
                filteredPoints.add(point);
            }
        }

        return filteredPoints;
    }

    private static double[] calculateLineOfBestFit(List<Point> points) {
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
        double intercept = (sumY - slope * sumX) / n;

        return new double[]{slope, intercept};
    }

    private static void drawLineSegment(Mat mat, double slope, double intercept, Scalar color) {

        Point start = new Point(0, intercept);

        Point end = new Point(mat.cols(), slope * mat.cols() + intercept);

        if (start.y < 0) {
            start.x = -intercept / slope;
            start.y = 0;
        } else if (start.y >= mat.rows()) {
            start.x = (mat.rows() - intercept) / slope;
            start.y = mat.rows() - 1;
        }

        if (end.y < 0) {
            end.x = -intercept / slope;
            end.y = 0;
        } else if (end.y >= mat.rows()) {
            end.x = (mat.rows() - intercept) / slope;
            end.y = mat.rows() - 1;
        }

        Imgproc.line(mat, start, end, color, 2);
    }

    private static Point findIntersection(double m1, double b1, double m2, double b2) {
        if (m1 == m2) {
            return null;
        }

        double x = (b2 - b1) / (m1 - m2);
        double y = m1 * x + b1;

        return new Point(x, y);
    }

    private static double calculateDistanceTolerance(double angleDegrees) {
        double angleRadians = Math.toRadians(angleDegrees);
        return 1.0 / Math.tan(angleRadians / 2.0);
    }

    private static Point findFurthestPointAlongSlope(List<Point> contourPoints, Point intersection, double slope, double tolerance) {
        Point furthestPoint = null;
        double maxDistance = -1;

        for (Point p : contourPoints) {
            double expectedY = slope * (p.x - intersection.x) + intersection.y;

            if (Math.abs(p.y - expectedY) < tolerance) {
                double distance = distance(p, intersection);
                if (distance > maxDistance) {
                    maxDistance = distance;
                    furthestPoint = p;
                }
            }
        }

        return furthestPoint;
    }

    private static double distance(Point p1, Point p2) {
        return Math.sqrt(Math.pow(p1.x - p2.x, 2) + Math.pow(p1.y - p2.y, 2));
    }



}
