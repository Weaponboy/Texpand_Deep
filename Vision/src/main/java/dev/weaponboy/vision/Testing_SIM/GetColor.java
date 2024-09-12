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
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;


public class GetColor implements VisionProcessor {

    ArrayList<Rect> redRects = new ArrayList<>();
    ArrayList<Rect> blueRects = new ArrayList<>();
    ArrayList<Rect> yellowRects = new ArrayList<>();

    Mat yellowMat = new Mat();
    Mat redMat = new Mat();
    Mat blueMat = new Mat();

    Paint red = new Paint();
    Paint blue = new Paint();
    Paint yellow = new Paint();

    Scalar blueLower = new Scalar(80,55,30);
    Scalar blueHigher = new Scalar(170,255,255);

    Scalar redLower = new Scalar(0,90,20);
    Scalar redHigher = new Scalar(10,255,255);

    Scalar yellowLower = new Scalar(10,110,110);
    Scalar yellowHigher = new Scalar(40,255,255);

    ArrayList<MatOfPoint> redContours = new ArrayList<>();
    ArrayList<MatOfPoint> yellowContours = new ArrayList<>();
    ArrayList<MatOfPoint> blueContours = new ArrayList<>();

    Mat redHierarchy = new Mat();
    Mat blueHierarchy = new Mat();
    Mat yellowHierarchy = new Mat();

    Point topLeft = new Point(0,0);
    Point topRight = new Point(0,0);

    Point bottomLeft = new Point(0,0);
    Point bottomRight = new Point(0,0);

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        clearRects();

        Imgproc.cvtColor(frame, yellowMat, COLOR_RGB2HSV);
        Imgproc.cvtColor(frame, redMat, COLOR_RGB2HSV);
        Imgproc.cvtColor(frame, blueMat, COLOR_RGB2HSV);

        inRange(blueMat, blueLower, blueHigher, blueMat);
        inRange(redMat, redLower, redHigher, redMat);
        inRange(yellowMat, yellowLower, yellowHigher, yellowMat);

        erode(blueMat, blueMat, new Mat(5, 5, CV_8U));
        erode(redMat, redMat, new Mat(5, 5, CV_8U));
        erode(yellowMat, yellowMat, new Mat(5, 5, CV_8U));

        dilate(blueMat, blueMat, new Mat(5, 5, CV_8U));
        dilate(redMat, redMat, new Mat(5, 5, CV_8U));
        dilate(yellowMat, yellowMat, new Mat(5, 5, CV_8U));

        findContours(blueMat, blueContours, blueHierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
        findContours(redMat, redContours, redHierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
        findContours(yellowMat, yellowContours, yellowHierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

//        for (int i = 0; i < blueContours.size(); i++){
//            Rect rect = boundingRect(blueContours.get(i));
//            blueRects.add(rect);
//        }
        for (int i = 0; i < redContours.size(); i++){
            Rect rect = boundingRect(redContours.get(i));
            redRects.add(rect);
        }
//        for (int i = 0; i < yellowContours.size(); i++){
//            Rect rect = boundingRect(yellowContours.get(i));
//            yellowRects.add(rect);
//        }

        for (MatOfPoint contour : redContours) {

            MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
            double epsilon = 0.02 * Imgproc.arcLength(contour2f, true);
            MatOfPoint2f approx = new MatOfPoint2f();
            Imgproc.approxPolyDP(contour2f, approx, epsilon, true);

            if (approx.total() == 4) {
                Point[] points = approx.toArray();

                Arrays.sort(points, Comparator.comparingDouble(p -> p.y));

//                topLeft = points[0];
//                topRight = points[1];
//
//                bottomLeft = points[2];
//                bottomRight = points[3];

                topLeft = points[0].x < points[1].x ? points[0] : points[1];
                topRight = points[0].x > points[1].x ? points[0] : points[1];

                bottomLeft = points[2].x < points[3].x ? points[2] : points[3];
                bottomRight = points[2].x > points[3].x ? points[2] : points[3];

//                Imgproc.line(frame, topLeft, topRight, new Scalar(0, 255, 0), 2);
//                Imgproc.line(frame, topLeft, bottomLeft, new Scalar(0, 255, 0), 2);
//                Imgproc.line(frame, topRight, bottomRight, new Scalar(0, 255, 0), 2);
//                Imgproc.line(frame, bottomLeft, bottomRight, new Scalar(0, 255, 0), 2);
            }
        }

//        redMat.copyTo(frame);

        blueContours.clear();
        yellowContours.clear();
        redContours.clear();
        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        setUpPaints(scaleCanvasDensity);

//        if (!blueRects.isEmpty()){
//            for (int i = 0; i < blueRects.size()-1; i++){
//                canvas.drawRect(makeGraphicsRect(blueRects.get(i), scaleBmpPxToCanvasPx), blue);
//            }
//        }
//
//        if (!yellowRects.isEmpty()){
//            for (int i = 0; i < yellowRects.size()-1; i++){
//                canvas.drawRect(makeGraphicsRect(yellowRects.get(i), scaleBmpPxToCanvasPx), yellow);
//            }
//        }

//        if (!redRects.isEmpty()){
//            for (int i = 0; i < redRects.size()-1; i++){
//                canvas.drawRect(makeGraphicsRect(redRects.get(i), scaleBmpPxToCanvasPx), red);
//            }
//        }

//        canvas.drawLine((float) topLeft.x * scaleBmpPxToCanvasPx, (float) topLeft.y * scaleBmpPxToCanvasPx, (float) topRight.x * scaleBmpPxToCanvasPx, (float) topRight.y * scaleBmpPxToCanvasPx, blue); // Top edge
//        canvas.drawLine((float) topLeft.x * scaleBmpPxToCanvasPx, (float) topLeft.y * scaleBmpPxToCanvasPx, (float) bottomLeft.x * scaleBmpPxToCanvasPx, (float) bottomLeft.y * scaleBmpPxToCanvasPx, blue); // Left edge
//        canvas.drawLine((float) topRight.x * scaleBmpPxToCanvasPx, (float) topRight.y * scaleBmpPxToCanvasPx, (float) bottomRight.x * scaleBmpPxToCanvasPx, (float) bottomRight.y * scaleBmpPxToCanvasPx, blue); // Right edge
//        canvas.drawLine((float) bottomLeft.x * scaleBmpPxToCanvasPx, (float) bottomLeft.y * scaleBmpPxToCanvasPx, (float) bottomRight.x * scaleBmpPxToCanvasPx, (float) bottomRight.y * scaleBmpPxToCanvasPx, blue); // Bottom edge

        canvas.drawText("test",  200, 200 , red);
//        canvas.drawCircle((float) topLeft.x, (float) topLeft.y, 5, blue); // Draw a small circle at top-left
//        canvas.drawCircle((float) topRight.x, (float) topRight.y, 5, blue); // Draw a small circle at top-right
//        canvas.drawCircle((float) bottomLeft.x, (float) bottomLeft.y, 5, blue); // Draw a small circle at bottom-left
//        canvas.drawCircle((float) bottomRight.x, (float) bottomRight.y, 5, blue);

        System.out.println(topLeft.x);
        System.out.println(topLeft.y);
//        System.out.println("bottomLeft: (" + bottomLeft.x + ", " + bottomLeft.y + ")");
//        System.out.println("bottomRight: (" + bottomRight.x + ", " + bottomRight.y + ")");
    }

    private android.graphics.Rect makeGraphicsRect(Rect rect, float scaleBmpPxToCanvasPx) {

        int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
        int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
        int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx);

        int bottom = top + Math.round(rect.height * scaleBmpPxToCanvasPx);

        return new android.graphics.Rect(left, top, right, bottom);
    }

    private void setUpPaints(float scaleCanvasDensity){
        red.setColor(Color.RED);
        red.setStyle(Paint.Style.STROKE);
        red.setStrokeWidth(scaleCanvasDensity * 4);

        blue.setColor(Color.BLUE);
        blue.setStyle(Paint.Style.STROKE);
        blue.setStrokeWidth(scaleCanvasDensity * 4);

        yellow.setColor(Color.YELLOW);
        yellow.setStyle(Paint.Style.STROKE);
        yellow.setStrokeWidth(scaleCanvasDensity * 4);
    }

    public void clearRects(){
        yellowRects.clear();
        redRects.clear();
        blueRects.clear();
    }
}
