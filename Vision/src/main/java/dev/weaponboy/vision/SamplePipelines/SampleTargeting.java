package dev.weaponboy.vision.SamplePipelines;

import static org.opencv.core.Core.inRange;
import static org.opencv.core.CvType.CV_8U;
import static org.opencv.imgproc.Imgproc.CHAIN_APPROX_SIMPLE;
import static org.opencv.imgproc.Imgproc.COLOR_RGB2HSV;
import static org.opencv.imgproc.Imgproc.boundingRect;
import static org.opencv.imgproc.Imgproc.dilate;
import static org.opencv.imgproc.Imgproc.erode;
import static org.opencv.imgproc.Imgproc.findContours;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

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

    public Scalar redLower = new Scalar(120.4, 103.4, 53.8);
    public Scalar redHigher = new Scalar(201,255,255);

    ArrayList<MatOfPoint> redContours = new ArrayList<>();
    Mat redHierarchy = new Mat();

    Size rectSize = new Size(280, 90);

    Point center = new Point(200, 200);

    double angle = 30.0;

    ArrayList<Point> topY = new ArrayList<>();
    Point rightX = new Point();

    final double maxHypotLength = 440;
    final double minHypotLength = 440;

    RotatedRect rotatedRect = new RotatedRect(center, rectSize, angle);

    double maxShort = 140;
    double minShort = 60;

    double minLong = 180;
    double maxLong = 400;

    Point centerDet = new Point();

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
        findContours(redMat, contours, hierarchy, Imgproc.RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        for (int i = 0; i < contours.size(); i++){
            Rect rect = boundingRect(contours.get(i));
            if (rect.area() > 7000 && rect.area() < 1000000){
                redContours.add(contours.get(i));
            }
        }

        if (!redContours.isEmpty()){
            for (MatOfPoint contour : redContours) {
                Imgproc.drawContours(frame, Arrays.asList(contour), -1, new Scalar(0, 255, 0), 2);
            }

            for (MatOfPoint contour : redContours) {
                centerDet =  findTopPosition(frame, contour);
                if (!(center == null)){
                    Imgproc.circle(frame, center, 4, new Scalar(0, 0, 255), -1);
                }
            }
        }

        redContours.clear();
        topY.clear();

        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
//        canvas.drawCircle(centerDet.x, center.y, );
    }

    public Point findTopPosition(Mat input, MatOfPoint Contour){
        Point CenterPoint = null;

        List<Point> contourPointsSorted = Arrays.asList(Contour.toArray());
        List<Point> contourPoints = Arrays.asList(Contour.toArray());

        contourPointsSorted.sort(Comparator.comparingDouble(p -> p.y));
        Point topPoint = contourPointsSorted.get(0);

        int centerIndex = contourPoints.indexOf(topPoint);

        List<Point> sublistBefore = new ArrayList<>(contourPoints.subList(contourPoints.size()-61, contourPoints.size()-1));
        List<Point> sublistAfter = new ArrayList<>(contourPoints.subList(centerIndex, 60));

        double[] line1 = calculateLineOfBestFit(sublistBefore);
        double slope1 = line1[0];
        double intercept1 = line1[1];

        double[] line2 = calculateLineOfBestFit(sublistAfter);
        double slope2 = line2[0];
        double intercept2 = line2[1];

        Point intersection = findIntersection(slope1, intercept1, slope2, intercept2);

        drawLineSegment(input, slope1, intercept1, new Scalar(255, 0, 0));
        drawLineSegment(input, slope2, intercept2,  new Scalar(0, 255, 255));
        Imgproc.circle(input, intersection, 4, new Scalar(0, 0, 255), -1);

        Point furthestPoint = findFurthestPointAlongSlope(contourPoints, intersection, slope1, calculateDistanceTolerance(15));
        Point furthestPoint2 = findFurthestPointAlongSlope(contourPoints, intersection, slope2, calculateDistanceTolerance(15));

//        if (!(furthestPoint == null)){
//            Imgproc.circle(input, furthestPoint, 4, new Scalar(0, 0, 255), -1);
//        }
//
//        if (!(furthestPoint == null)){
//            Imgproc.circle(input, furthestPoint2, 4, new Scalar(0, 0, 255), -1);
//        }

        assert furthestPoint != null;
        double deltaXFirst = Math.abs(furthestPoint.x - intersection.x);
        double deltaYFirst = Math.abs(furthestPoint.y - intersection.y);
        double firstLength = Math.hypot(deltaXFirst, deltaYFirst);
        Imgproc.putText(input, String.valueOf(firstLength), new Point(200, 200), 2, 1, new Scalar(0, 255, 0), 4, 2, false);

        assert furthestPoint2 != null;
        double deltaXSecond = Math.abs(furthestPoint2.x - intersection.x);
        double deltaYSecond = Math.abs(furthestPoint2.y - intersection.y);
        double secondLength = Math.hypot(deltaYSecond, deltaXSecond);
        Imgproc.putText(input, String.valueOf(secondLength), new Point(200, 240), 2, 1, new Scalar(0, 255, 0), 4, 2, false);


        if ((secondLength < maxShort && secondLength > minShort) && (firstLength < maxLong && firstLength > minLong)){
            CenterPoint = new Point(intersection.x - (deltaXSecond/2) + (deltaXFirst/2), intersection.y +  (deltaYFirst/2) + (deltaYSecond/2));
        }else if ((firstLength < maxShort && firstLength > minShort) && (secondLength < maxLong && secondLength > minLong)){
            CenterPoint = new Point(intersection.x - (deltaXSecond/2) + (deltaXFirst/2), intersection.y +  (deltaYFirst/2) + (deltaYSecond/2));
        }

        return CenterPoint;
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
