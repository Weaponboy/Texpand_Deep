package dev.weaponboy.vision.SamplePipelines;

import static org.opencv.core.Core.inRange;
import static org.opencv.core.CvType.CV_8U;
import static org.opencv.imgproc.Imgproc.COLOR_RGB2HSV;
import static org.opencv.imgproc.Imgproc.boundingRect;
import static org.opencv.imgproc.Imgproc.dilate;
import static org.opencv.imgproc.Imgproc.erode;
import static org.opencv.imgproc.Imgproc.findContours;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
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
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicReference;

public class SampleTargeting  implements VisionProcessor, CameraStreamSource {

    private final AtomicReference<Bitmap> lastFrame =
            new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

    Mat redMat = new Mat();

    public Scalar blueLower = new Scalar(103.4, 114, 52);
    public Scalar blueHigher = new Scalar(119, 255, 255);

    public Scalar yellowLower = new Scalar(0, 80.8, 126.1);
    public Scalar yellowHigher = new Scalar(26, 255, 255);

    public Scalar redLower = new Scalar(9, 40, 160);
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

    public int getDetectionCounter() {
        return detectionCounter;
    }

    public void resetDetectionCounter() {
        this.detectionCounter = 0;
    }

    int detectionCounter = 0;

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

    Rect ROI = new Rect(0, 0, 1000, 960);

    @Override
    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
        continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos){

        Imgproc.cvtColor(frame, redMat, COLOR_RGB2HSV);

        inRange(redMat, redLower, redHigher, redMat);

        erode(redMat, redMat, new Mat(5, 5, CV_8U));

        dilate(redMat, redMat, new Mat(5, 5, CV_8U));

        ArrayList<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();

        Imgproc.findContours(redMat, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);

        for (int i = 0; i < contours.size(); i++){
            Rect rect = boundingRect(contours.get(i));
            if (rect.area() > 30000 && rect.area() < 120000){
                redContoursSingle.add(contours.get(i));
                redRectsSingle.add(rect);
            } else if (rect.area() > 120000){
                redContoursMulti.add(contours.get(i));
                redRectsMulti.add(rect);
            }
        }

        if (!redContoursSingle.isEmpty()){

            rotate = true;

            ArrayList<Point> centerPoints = new ArrayList<>();

            for (MatOfPoint contour: redContoursSingle){
                Moments moments = Imgproc.moments(contour);

                double cx = moments.get_m10() / moments.get_m00();
                double cy = moments.get_m01() / moments.get_m00();

                centerPoints.add(new Point(cx, cy));
            }

            int closestIndex = findClosestPoint(centerPoints, new Point(640, 480));

            Moments moments = Imgproc.moments(redContoursSingle.get(closestIndex));

            double cx = moments.get_m10() / moments.get_m00();
            double cy = moments.get_m01() / moments.get_m00();

            avCounter++;
            pointsAve.add(new Point(cx, cy));
            MatOfPoint2f contour2f = new MatOfPoint2f(redContoursSingle.get(closestIndex).toArray());

            RotatedRect rotatedRect = Imgproc.minAreaRect(contour2f);

            double angle = rotatedRect.angle;
            Size rectSize = rotatedRect.size;

            // Determine the long side of the rectangle
            boolean isWidthLonger = rectSize.width > rectSize.height;

            if (isWidthLonger) {
                ratio = rotatedRect.size.width/rotatedRect.size.height;
            }else {
                angle = angle + 90;
                ratio = rotatedRect.size.height/rotatedRect.size.width;
            }

            if (angle > 90) {
                angle -= 180; // Normalize to the range [-90, 90]
            }

            angleRotate = -angle;

            center = new Point(cx, cy);

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

            avCounter++;
            pointsAve.add(center);

        } else if (!redContoursMulti.isEmpty() && redContoursSingle.isEmpty()) {

            rotate = true;

            for (int i = 0; i < redContoursMulti.size(); i++) {

                Moments moments = Imgproc.moments(redContoursMulti.get(i));

                double cx = moments.get_m10() / moments.get_m00();
                double cy = moments.get_m01() / moments.get_m00();

                contourCenter = new Point(cx, cy);
                MatOfPoint2f contour2f = new MatOfPoint2f(redContoursMulti.get(i).toArray());

                RotatedRect rotatedRect = Imgproc.minAreaRect(contour2f);

                rectCenter = rotatedRect.center;

                double angle = rotatedRect.angle;
                Size rectSize = rotatedRect.size;

                // Determine the long side of the rectangle
                boolean isWidthLonger = rectSize.width > rectSize.height;

                // Adjust the angle to ensure it's based on the long side of the rectangle
                if (isWidthLonger) {
                    angle = 90 - angle; // Add 90 degrees if height is the longer side
                }else {
                    angle = -(angle); // Add 90 degrees if height is the longer side
                }

                // Normalize the angle:
                // Positive angle = tipped right, Negative angle = tipped left
                if (angle > 90) {
                    angle -= 180; // Normalize to the range [-90, 90]
                }

                angleRotate = angle;

                Point[] rectPoints = new Point[4];
                Point[] rectPointsDraw = new Point[4];
                rotatedRect.points(rectPoints);
                rotatedRect.points(rectPointsDraw);  // Fills the array with the corner points// Fills the array with the corner points

                Point p1 = rectPoints[0];      // First point of the side
                Point p2 = rectPoints[(1) % 4];  // Second point of the side (next in the loop)

                if (!isWidthLonger){
                    p1 = rectPoints[0];
                    p2 = rectPoints[(1) % 4];
                }else {
                    p1 = rectPoints[1];
                    p2 = rectPoints[(2) % 4];
                }

                double deltaY = p2.y - p1.y;
                double deltaX = p2.x - p1.x;

                if (deltaX == 0) {
                    return Double.POSITIVE_INFINITY;  // Vertical line has an infinite slope
                }

                double slope = deltaY / deltaX;

                double hypot;

                if (isWidthLonger){
                    hypot = rectSize.width;
                }else {
                    hypot = rectSize.height;
                }

                if (hypot > 350){

                }else {
                    if (isWidthLonger){
                        hypot = rectSize.height;
                    }else {
                        hypot = rectSize.width;
                    }
                }

                Point deltaPoint = calculateDeltas(slope, (hypot/2)-60);

                boolean isBottom = findIsBottom(rotatedRect, contourCenter);

                Imgproc.putText(frame, String.valueOf(isBottom), new Point(20, 160), 2, 1, new Scalar(0, 255, 0), 4, 2, false);


                if (isBottom && slope > 0){
                    center = new Point(rectCenter.x + deltaPoint.x,  rectCenter.y + deltaPoint.y);
                } else if (isBottom && slope < 0) {
                    center = new Point(rectCenter.x - deltaPoint.y,  rectCenter.y + deltaPoint.x);
                }else if (isBottom) {
                    center = new Point(rectCenter.x - deltaPoint.x,  rectCenter.y - deltaPoint.y);
                } else {
                    center = new Point(rectCenter.x + deltaPoint.x,  rectCenter.y + deltaPoint.y);
                }

//                contourCenter = deltaPoint;

                avCounter++;
                pointsAve.add(center);

                // Convert the points to a list for drawing the contour
                MatOfPoint points = new MatOfPoint(rectPointsDraw);

                // Draw the rotated rectangle using drawContours
                Imgproc.drawContours(frame,
                        java.util.Collections.singletonList(points),
                        -1,   // Index of contour (only one here)
                        new Scalar(255, 255, 0),  // Color in BGR format (blue)
                        2);

            }

        }else{
            Imgproc.rectangle(frame, new Rect(200, 200, 220, 220), new Scalar(0, 255, 0));
            avCounter = 0;
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

        Imgproc.circle(frame, center, 6, new Scalar(0, 255, 0), -1);

        Imgproc.putText(frame, String.valueOf(railTarget), new Point(20, 40), 2, 1, new Scalar(0, 255, 0), 4, 2, false);
        Imgproc.putText(frame, String.valueOf(slidesDelta), new Point(20, 80), 2, 1, new Scalar(0, 255, 0), 4, 2, false);
        Imgproc.putText(frame, String.valueOf(angleRotate), new Point(20, 120), 2, 1, new Scalar(0, 255, 0), 4, 2, false);
//        Imgproc.putText(frame, String.valueOf(ratio), new Point(20, 160), 2, 1, new Scalar(0, 255, 0), 4, 2, false);

        Bitmap b = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(frame, b);
        lastFrame.set(b);

        return null;
    }

    public int findClosestPoint(ArrayList<Point> points, Point center) {
        if (points == null || points.isEmpty() || center == null) {
            throw new IllegalArgumentException("Invalid input.");
        }

        int closestIndex = -1;
        double minDistance = Double.MAX_VALUE;

        for (int i = 0; i < points.size(); i++) {
            double distance = euclideanDistance(points.get(i), center);
            if (distance < minDistance) {
                minDistance = distance;
                closestIndex = i;
            }
        }

        return closestIndex;
    }

    private static double euclideanDistance(Point p1, Point p2) {
        return Math.sqrt(Math.pow(p1.x - p2.x, 2) + Math.pow(p1.y - p2.y, 2));
    }

    public static Point calculateDeltas(double slope, double hypotenuse) {
        // Calculate the angle θ using the slope
        double theta = Math.atan(slope);  // θ = atan(m)

        // Calculate Δx and Δy using the hypotenuse
        double deltaX = hypotenuse * Math.cos(theta);  // Δx = hypotenuse * cos(θ)
        double deltaY = hypotenuse * Math.sin(theta);  // Δy = hypotenuse * sin(θ)

        return new Point(deltaX, deltaY);
    }

    public boolean findIsBottom(RotatedRect rect, Point p) {
        // Get the 4 corner points of the rotated rectangle
        Point[] rectPoints = new Point[4];
        rect.points(rectPoints);

        // Rotate the point into the rectangle's coordinate system
        Point rectCenter = rect.center;
        double angle = Math.toRadians(rect.angle); // Convert to radians

        // Translate the point to the rectangle's center
        double translatedX = p.x - rectCenter.x;
        double translatedY = p.y - rectCenter.y;

        // Rotate the point by the rectangle's angle (inverse rotation)
        double rotatedX = translatedX * Math.cos(-angle) - translatedY * Math.sin(-angle);
        double rotatedY = translatedX * Math.sin(-angle) + translatedY * Math.cos(-angle);

        // Rectangle dimensions
        double halfWidth = rect.size.width / 2;
        double halfHeight = rect.size.height / 2;

        // Check if the point lies near the left, right, top, or bottom side
        if (Math.abs(rotatedX) > halfWidth || Math.abs(rotatedY) > halfHeight) {

        }

        // Compare the point's position relative to the edges
        double leftDistance = Math.abs(-halfWidth - rotatedX);
        double rightDistance = Math.abs(halfWidth - rotatedX);
        double topDistance = Math.abs(-halfHeight - rotatedY);
        double bottomDistance = Math.abs(halfHeight - rotatedY);

        if (topDistance > bottomDistance){
            return true;
        }else {
            return false;
        }

    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        red.setColor(Color.BLUE);
        red.setStyle(Paint.Style.STROKE);
        red.setStrokeWidth(scaleCanvasDensity * 4);

        if (avCounter > 5){
            Point center = findAve(pointsAve, avCounter);
            canvas.drawCircle((float) center.x*scaleBmpPxToCanvasPx, (float) center.y*scaleBmpPxToCanvasPx, 4, red);
            avCounter = 0;
            pointsAve.clear();

            double stuff = ((getHeight()-center.y) / pixelsToCmWidth);
            double stuff2 = ((getWidth() -  center.x) / pixelsToCmHeight)-((double) 46 /2) - 6;

            railTarget = stuff-7;
            slidesDelta = stuff2;

            detectionCounter++;

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
