package dev.weaponboy.vision.SamplePipelines;

import static org.opencv.core.Core.inRange;
import static org.opencv.core.Core.mean;
import static org.opencv.core.CvType.CV_8U;
import static org.opencv.imgproc.Imgproc.COLOR_RGB2BGR;
import static org.opencv.imgproc.Imgproc.COLOR_RGB2HSV;
import static org.opencv.imgproc.Imgproc.boundingRect;
import static org.opencv.imgproc.Imgproc.dilate;
import static org.opencv.imgproc.Imgproc.erode;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Core;
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
import java.util.concurrent.atomic.AtomicReference;

import dev.weaponboy.vision.detectionData;
import dev.weaponboy.nexus_pathing.PathingUtility.RobotPower;

public class findAngleUsingContour implements VisionProcessor, CameraStreamSource {

    private final AtomicReference<Bitmap> lastFrame =
            new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

    public ArrayList<detectionData> detections = new ArrayList<>();

    ArrayList<detectionData> currentDetections = new ArrayList<>();
    ArrayList<detectionData> detectionsLastLoop = new ArrayList<>();
    ArrayList<detectionData> detectionsBeforeLastLoop = new ArrayList<>();

    Mat redMat = new Mat();
    Mat redMat2 = new Mat();

//    public Scalar redLower = new Scalar(0, 148, 36);
//    public Scalar redHigher = new Scalar(9,255,255);

    public Scalar redLower = new Scalar(0, 0, 164);
    public Scalar redHigher = new Scalar(222,160,255);

    public Scalar redLower2 = new Scalar(160, 25, 123);
    public Scalar redHigher2 = new Scalar(220,255,255);

    public Scalar blueLower = new Scalar(40, 60, 60);
    public Scalar blueHigher = new Scalar(160,255,255);

    public Scalar yellowLower = new Scalar(9, 145, 134);
    public Scalar yellowHigher = new Scalar(38,255,255);

    ArrayList<MatOfPoint> redContoursSingle = new ArrayList<>();
    ArrayList<Rect> redRectsSingle = new ArrayList<>();

    Rect ROI = new Rect(450, 250, 400, 300);

    Point TargetPoint;
    double angle = 0;

    private final double viewSizeCMXAxis = 70;
    private final double viewSizeCMYAxis = 53;

    int loopCounter = 0;

    public boolean isScanning() {
        return scanning;
    }

    public void setScanning(boolean scanning) {
        this.scanning = scanning;
    }

    boolean scanning = true;

    public Mat frameSub;

    public enum TargetColor{
        red,
        blue,
        yellow
    }

    public boolean closestFirst = false;

    public TargetColor getTargetColor() {
        return targetColor;
    }

    public void setTargetColor(TargetColor targetColor) {
        this.targetColor = targetColor;
    }

    TargetColor targetColor = TargetColor.yellow;

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
    }

    @Override
    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
        continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos){

        frameSub = frame.submat(ROI);

        if (scanning && loopCounter <= 2){

            loopCounter++;
            detectionsLastLoop = currentDetections;
            detectionsBeforeLastLoop = detectionsLastLoop;
            currentDetections.clear();

            ArrayList<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();

            if (targetColor == TargetColor.red){
                Imgproc.cvtColor(frameSub, redMat, COLOR_RGB2BGR);

                inRange(redMat, redLower, redHigher, redMat);

                erode(redMat, redMat, new Mat(5, 5, CV_8U));

                dilate(redMat, redMat, new Mat(5, 5, CV_8U));

                Imgproc.findContours(redMat, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);

            } else if (targetColor == TargetColor.blue) {
                Imgproc.cvtColor(frameSub, redMat, COLOR_RGB2HSV);

                inRange(redMat, blueLower, blueHigher, redMat);

                erode(redMat, redMat, new Mat(5, 5, CV_8U));

                dilate(redMat, redMat, new Mat(5, 5, CV_8U));

                Imgproc.findContours(redMat, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);

            } else if (targetColor == TargetColor.yellow) {
                Imgproc.cvtColor(frameSub, redMat, COLOR_RGB2HSV);

                inRange(redMat, yellowLower, yellowHigher, redMat);

                erode(redMat, redMat, new Mat(5, 5, CV_8U));

                dilate(redMat, redMat, new Mat(5, 5, CV_8U));

                Imgproc.findContours(redMat, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);

            }

            for (int i = 0; i < contours.size(); i++){
                Rect rect = boundingRect(contours.get(i));
                if (rect.area() > 1000 && rect.area() < 5000){
                    redContoursSingle.add(contours.get(i));
                    redRectsSingle.add(rect);
                }
            }

            ArrayList<Point> centerPointsSingle = new ArrayList<>();

            for (int i = 0; i < redContoursSingle.size(); i++){
                centerPointsSingle.add(getContourCenter(redContoursSingle.get(i)));
            }

            ArrayList<MatOfPoint> sortedContoursSingle;

            if (closestFirst){
                sortedContoursSingle = sortContoursByYClosest(redContoursSingle, centerPointsSingle);
            }else{
                sortedContoursSingle = sortContoursByY(redContoursSingle, centerPointsSingle);
            }

            for (MatOfPoint contour : sortedContoursSingle) {
                Imgproc.drawContours(frameSub, Arrays.asList(contour), -1, new Scalar(0, 255, 0), 2);
            }

            if (!sortedContoursSingle.isEmpty()) {

                currentDetections.clear();

//                MatOfPoint Contour = sortedContoursSingle.get(sortedContoursSingle.size() - 1);

                for (MatOfPoint Contour: sortedContoursSingle){
                    Rect rect = boundingRect(Contour);

                    if (rect.area() > 1000 && rect.area() < 5000) {

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

                        double angleMultiplier = (0.9) * (Math.abs(angle)) / 100;

                        if (angleMultiplier > 0) {
                            angleMultiplier = 1 - angleMultiplier;
                        } else if (angleMultiplier < 0) {
                            angleMultiplier = -(1 - Math.abs(angleMultiplier));
                        }

                        double x = calculateAdjustment(contourCenter.y, 496, 0, 0, 10);

                        TargetPoint = new Point(contourCenter.x, contourCenter.y - x);

                        currentDetections.add(new detectionData(System.nanoTime(), TargetPoint, angle));

                        Imgproc.circle(frameSub, TargetPoint, 5, new Scalar(255, 0, 0), -1);
                        Imgproc.putText(frameSub, String.valueOf((int) angle), TargetPoint, 2, 1, new Scalar(0, 255, 0), 4, 2, false);

                    } else if (rect.area() > 5000 && rect.area() < 150000) {

                        MatOfPoint closestContour = Contour;

                        Point lowestPoint = getLowestYPoint(closestContour);
                        Point contourCenter = getContourCenter(closestContour);

                        List<Point> contourPoints = closestContour.toList();

                        List<Point> angleArray = null;

                        if (lowestPoint.x > contourCenter.x) {
                            int index = contourPoints.indexOf(lowestPoint);
                            Point before;

                            before = contourPoints.get(index + 5);

                            if (before.x > lowestPoint.x) {
                                if (index > 22) {
                                    angleArray = contourPoints.subList(index, index - 20);
                                } else {
                                    angleArray = contourPoints.subList(contourPoints.size() - 20, contourPoints.size() - 1);
                                }

                            } else {
                                if (index + 22 < contourPoints.size()) {
                                    angleArray = contourPoints.subList(index, index + 20);
                                } else {
                                    angleArray = contourPoints.subList(0, 20);
                                }
                            }

                        } else if (lowestPoint.x < contourCenter.x) {
                            int index = contourPoints.indexOf(lowestPoint);
                            Point before, after;

                            if (index > 5) {
                                before = contourPoints.get(index - 5);
                            } else {
                                before = contourPoints.get(contourPoints.size() - 6);
                            }

                            if (before.x > lowestPoint.x) {
                                if (index > 42) {
                                    angleArray = contourPoints.subList(index - 20, index - 40);
                                } else {
                                    angleArray = contourPoints.subList(contourPoints.size() - 40, contourPoints.size() - 21);
                                }

                            } else {
                                if (index + 22 < contourPoints.size()) {
                                    angleArray = contourPoints.subList(index, index + 20);
                                } else {
                                    angleArray = contourPoints.subList(0, 20);
                                }
                            }
                        }

                        double slope;

                        if (angleArray != null) {
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
                            double yError = -4;

                            if (angle < 0) {
                                angle = 135 + (-angle);
                                angleRadians = Math.toRadians(angle);
                            } else {
                                angle = 230 + (-angle);
                                angleRadians = Math.toRadians(angle);
                            }

                            angle = Math.toDegrees(Math.atan(slope));

                            this.angle = angle;

                            double angleMultiplier = ((0.9) * (angle) / 100);
                            double angleMultiplierY = (0.9) * (Math.abs(angle)) / 100;

                            if (angleMultiplier > 0) {
                                angleMultiplier = 1 - angleMultiplier;
                            } else if (angleMultiplier < 0) {
                                angleMultiplier = -(1 - Math.abs(angleMultiplier));
                            }

                            relativeXCorrective = (yError) * Math.sin(angleRadians) + (xError) * Math.cos(angleRadians);
                            relativeYCorrective = (yError) * Math.cos(angleRadians) - (xError) * Math.sin(angleRadians);

                            Point centerPoint = new Point();

                            if (intercept != null){
                                centerPoint = new Point(intercept.x + relativeXCorrective, intercept.y + relativeYCorrective);
                            }

                            double x = calculateAdjustment(centerPoint.y, 496, 0, 0, 20);
                            double y = calculateAdjustment(centerPoint.y, 496, 0, 0, 300);

                            if (angle < 60 && angle > -60) {
                                xError = x * angleMultiplier;
                                yError = y * angleMultiplierY;
                            } else {
                                yError = 0;
                            }

                            if (intercept != null){
                                TargetPoint = new Point(intercept.x + relativeXCorrective + xError, intercept.y + relativeYCorrective + yError);

                                currentDetections.add(new detectionData(System.nanoTime(), TargetPoint, angle));

                                Imgproc.circle(frameSub, TargetPoint, 5, new Scalar(255, 0, 0), -1);
                                Imgproc.putText(frameSub, String.valueOf((int) angle), TargetPoint, 2, 1, new Scalar(0, 255, 0), 4, 2, false);

                            }

                        } else {
                            Imgproc.putText(frameSub, "Bad", new Point(20, 600), 2, 4, new Scalar(0, 255, 0), 4, 2, false);
                        }

                    }

                }

            }

            contours.clear();

            if (loopCounter == 2) {
                if ((currentDetections.size() != detectionsLastLoop.size())) {
                    detections.clear();
                    loopCounter = 0;
                    scanning = true;
                } else {
                    detections.clear();

                    detections = currentDetections;

//                    for (int i = 0; i < currentDetections.size(); i++) {
//                        detections.add(new detectionData(System.nanoTime(), new Point((currentDetections.get(i).getTargetPoint().x + detectionsLastLoop.get(i).getTargetPoint().x) / 2, (currentDetections.get(i).getTargetPoint().y + detectionsLastLoop.get(i).getTargetPoint().y) / 2), (currentDetections.get(i).getAngle() + detectionsLastLoop.get(i).getAngle()) / 2));
//                    }

                    scanning = false;
                }
            }

        }

        if (loopCounter == 4){
            scanning = true;
            loopCounter = 0;
        }

        if (loopCounter > 2){
            loopCounter++;
        }

        redContoursSingle.clear();
        redRectsSingle.clear();

        Bitmap b = Bitmap.createBitmap(frameSub.width(), frameSub.height(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(frameSub, b);
        lastFrame.set(b);

        redMat.release();
        frameSub.release();

        return null;
    }

    public ArrayList<detectionData> convertPositionsToFieldPositions(RobotPower power, double slidePosition){

        //first triangle
        double pivotHeight = slidePosition + 43;
        double firstHypot = Math.sqrt((pivotHeight*pivotHeight)+(29 * 29));

        double angleA = Math.toDegrees(Math.atan(29/pivotHeight));
        double angleB = Math.toDegrees(Math.atan(pivotHeight/29));
        double angleC = 90;

        //second triangle
        double angleD = 81;
        double angleE = Math.toDegrees(Math.atan(8 / firstHypot));
        double angleF = 180 - angleE - angleD;

        double sideD = firstHypot;
        double sideE = 8;
        double sideF = sideD * (Math.sin(Math.toRadians(angleF)) / Math.sin(Math.toRadians(angleD)));

        //vision triangle
        double angleG = 34.6;
        double angleH = 180 - angleB - angleE;
        double angleI = 180 - angleH - angleG;

        double sideI = sideF;
        double sideG = sideI * (Math.sin(Math.toRadians(angleG)) / Math.sin(Math.toRadians(angleI)));
        double sideH = sideI * (Math.sin(Math.toRadians(angleH)) / Math.sin(Math.toRadians(angleI)));

        //top vision triangle
        double angleJ = angleG;
        double angleK = (180 - angleJ)/2;
        double angleL = (180 - angleJ)/2;

        double sideL = sideI;
        double sideJ = sideL * (Math.sin(Math.toRadians(angleJ)) / Math.sin(Math.toRadians(angleL)));
        double sideK = sideL;

        //bottom vision triangle
        double angleM = angleH - angleK;
        double angleN = 180 - angleL;
        double angleO = angleI;

        double sideN = sideG;
        double sideO = sideJ;
        double sideM = sideN * (Math.sin(Math.toRadians(angleM)) / Math.sin(Math.toRadians(angleN)));

        ArrayList<detectionData> fieldDetections = new ArrayList<>();

        for (detectionData detection: detections){

            double centerPoint = 190;

            double yExtra = calculateAdjustment(detection.getTargetPoint().y, 300, 1, 0, 1.609);

//            yExtra = 1;

            if (detection.getTargetPoint().x < 180 || detection.getTargetPoint().x > 200){

            }else {
                yExtra = 1;
            }

            double pixelsToCMRelX = sideJ /ROI.height;
            double pixelsToCMRelY = viewSizeCMYAxis/ROI.width;

            double relYPosition = 0;
            if (detection.getTargetPoint().x > centerPoint){

                double updatedPosition = detection.getTargetPoint().x - centerPoint * yExtra;

                relYPosition = ((detection.getTargetPoint().x-centerPoint)*pixelsToCMRelY) * yExtra;

            } else if (detection.getTargetPoint().x < centerPoint) {

                double updatedPosition = detection.getTargetPoint().x - centerPoint * yExtra;

                relYPosition = ((detection.getTargetPoint().x  - centerPoint)*pixelsToCMRelY) * yExtra;

            }

            double relXPosition = ((((ROI.height - detection.getTargetPoint().y))*pixelsToCMRelX));

            double targetPoint = relXPosition;

            double miniTriangleSide = Math.sqrt(Math.pow(targetPoint, 2) + Math.pow(sideI, 2) - 2 * targetPoint * sideI * Math.cos(Math.toRadians(angleK)));
            double angleMini = Math.toDegrees(targetPoint * Math.sin(Math.toRadians(angleK)) / miniTriangleSide);
            double otherAngle = 180 - angleMini - angleK;

            double invertedOtherAngle = 180 - otherAngle;

            double otherInsideAngle = 180 - invertedOtherAngle - angleM;

            double realWorldPosition = targetPoint * (Math.sin(Math.toRadians(invertedOtherAngle)) / Math.sin(Math.toRadians(otherInsideAngle)));

            relXPosition = realWorldPosition + 22;

//            relXPosition = sideN;

            double globalX = relXPosition * Math.cos(Math.toRadians(power.getPivot())) - relYPosition * Math.sin(Math.toRadians(power.getPivot()));
            double globalY = relXPosition * Math.sin(Math.toRadians(power.getPivot())) + relYPosition * Math.cos(Math.toRadians(power.getPivot()));

            if (relYPosition < 13 && relYPosition > -13 && (realWorldPosition - 17.5 - 10) < 45){
                fieldDetections.add(new detectionData(detection.getReadTime(), new Point(power.getVertical()+globalX, power.getHorizontal()+globalY), detection.getAngle()));

            }

        }

        return fieldDetections;
    }

    public ArrayList<detectionData> convertPositionsToFieldPositions(RobotPower power, double slidePosition, double pivotAngle){

        //first triangle
        double pivotHeight = slidePosition + 43;
        double firstHypot = Math.sqrt((pivotHeight*pivotHeight)+(29 * 29));

        double angleA = Math.toDegrees(Math.atan(29/pivotHeight));
        double angleB = Math.toDegrees(Math.atan(pivotHeight/29));
        double angleC = 90;

        //second triangle
        double angleD = 80;
        double angleE = Math.toDegrees(Math.atan(8 / firstHypot));
        double angleF = 180 - angleE - angleD;

        double sideD = firstHypot;
        double sideE = 8;
        double sideF = sideD * (Math.sin(Math.toRadians(angleF)) / Math.sin(Math.toRadians(angleD)));

        //vision triangle
        double angleG = 34.6;
        double angleH = 180 - angleB - angleE;
        double angleI = 180 - angleH - angleG;

        double sideI = sideF;
        double sideG = sideI * (Math.sin(Math.toRadians(angleG)) / Math.sin(Math.toRadians(angleI)));
        double sideH = sideI * (Math.sin(Math.toRadians(angleH)) / Math.sin(Math.toRadians(angleI)));

        //top vision triangle
        double angleJ = angleG;
        double angleK = (180 - angleJ)/2;
        double angleL = (180 - angleJ)/2;

        double sideL = sideI;
        double sideJ = sideL * (Math.sin(Math.toRadians(angleJ)) / Math.sin(Math.toRadians(angleL)));
        double sideK = sideL;

        //bottom vision triangle
        double angleM = angleH - angleK;
        double angleN = 180 - angleL;
        double angleO = angleI;

        double sideN = sideG;
        double sideO = sideJ;
        double sideM = sideN * (Math.sin(Math.toRadians(angleM)) / Math.sin(Math.toRadians(angleN)));

        ArrayList<detectionData> fieldDetections = new ArrayList<>();

        for (detectionData detection: detections){

            double centerPoint = 190;

            double yExtra = calculateAdjustment(detection.getTargetPoint().y, 300, 1, 0, 1.64);

//            yExtra = 1;

            if (detection.getTargetPoint().x < 180 || detection.getTargetPoint().x > 200){

            }else {
                yExtra = 1;
            }

            double pixelsToCMRelX = sideJ /ROI.height;
            double pixelsToCMRelY = viewSizeCMYAxis/ROI.width;

            double relYPosition = 0;

            if (detection.getTargetPoint().x > centerPoint){

                double updatedPosition = detection.getTargetPoint().x - centerPoint * yExtra;

                relYPosition = ((detection.getTargetPoint().x-centerPoint)*pixelsToCMRelY) * yExtra;

            } else if (detection.getTargetPoint().x < centerPoint) {

                double updatedPosition = detection.getTargetPoint().x - centerPoint * yExtra;

                relYPosition = ((detection.getTargetPoint().x  - centerPoint)*pixelsToCMRelY) * yExtra;

            }

            double relXPosition = ((((ROI.height - detection.getTargetPoint().y))*pixelsToCMRelX));

            double targetPoint = relXPosition;

            double miniTriangleSide = Math.sqrt(Math.pow(targetPoint, 2) + Math.pow(sideI, 2) - 2 * targetPoint * sideI * Math.cos(Math.toRadians(angleK)));
            double angleMini = Math.toDegrees(targetPoint * Math.sin(Math.toRadians(angleK)) / miniTriangleSide);
            double otherAngle = 180 - angleMini - angleK;

            double invertedOtherAngle = 180 - otherAngle;

            double otherInsideAngle = 180 - invertedOtherAngle - angleM;

            double realWorldPosition = targetPoint * (Math.sin(Math.toRadians(invertedOtherAngle)) / Math.sin(Math.toRadians(otherInsideAngle)));

            relXPosition = realWorldPosition + 22.5;

//            relXPosition = sideN;

            double globalX = relXPosition * Math.cos(Math.toRadians(power.getPivot())) - relYPosition * Math.sin(Math.toRadians(power.getPivot()));
            double globalY = relXPosition * Math.sin(Math.toRadians(power.getPivot())) + relYPosition * Math.cos(Math.toRadians(power.getPivot()));

            if (relYPosition < 13 && relYPosition > -13 && (relXPosition - 22.5 - 10) < 55 && (relXPosition - 22.5 - 10) > 5){
                fieldDetections.add(new detectionData(detection.getReadTime(), new Point(power.getVertical()+globalX, power.getHorizontal()+globalY), detection.getAngle()));
            }

        }

        return fieldDetections;
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
            sortedContours.add(contours.get(indices.get(i)));
            if (centerPoints.get(indices.get(i)).x > 80 && centerPoints.get(indices.get(i)).x < 550){

            }
        }

        return sortedContours;
    }

    public ArrayList<MatOfPoint> sortContoursByYClosest(ArrayList<MatOfPoint> contours, ArrayList<Point> centerPoints) {
        // Create a list of indices for sorting
        List<Integer> indices = new ArrayList<>();
        for (int i = 0; i < centerPoints.size(); i++) {
            indices.add(i);
        }

        // Sort indices in reverse order based on Y-coordinate of center points
        Collections.sort(indices, new Comparator<Integer>() {
            @Override
            public int compare(Integer index1, Integer index2) {
                return Double.compare(centerPoints.get(index2).y, centerPoints.get(index1).y); // Reverse comparison
            }
        });

        // Create a list for sorted contours
        ArrayList<MatOfPoint> sortedContours = new ArrayList<>();

        // Filter and add contours to the sorted list
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