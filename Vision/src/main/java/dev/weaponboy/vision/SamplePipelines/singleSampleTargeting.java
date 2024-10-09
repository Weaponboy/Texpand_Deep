package dev.weaponboy.vision.SamplePipelines;

import static org.opencv.core.Core.inRange;
import static org.opencv.core.CvType.CV_8U;
import static org.opencv.imgproc.Imgproc.CHAIN_APPROX_SIMPLE;
import static org.opencv.imgproc.Imgproc.COLOR_RGB2HSV;
import static org.opencv.imgproc.Imgproc.RETR_TREE;
import static org.opencv.imgproc.Imgproc.boundingRect;
import static org.opencv.imgproc.Imgproc.dilate;
import static org.opencv.imgproc.Imgproc.erode;
import static org.opencv.imgproc.Imgproc.findContours;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;


public class singleSampleTargeting implements VisionProcessor, CameraStreamSource {

    private final AtomicReference<Bitmap> lastFrame =
            new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

    Telemetry telemetry;

    public singleSampleTargeting(){

    }

    public singleSampleTargeting(Telemetry telemetry){
        this.telemetry = telemetry;
    }

    ArrayList<Rect> redRects = new ArrayList<>();
    ArrayList<Rect> blueRects = new ArrayList<>();
    ArrayList<Rect> yellowRects = new ArrayList<>();

    ArrayList<MatOfPoint> sortedRedContours = new ArrayList<>();
//    ArrayList<MatOfPoint> yellowContours = new ArrayList<>();
//    ArrayList<MatOfPoint> blueContours = new ArrayList<>();

    Mat yellowMat = new Mat();
    Mat redMat = new Mat();
    Mat blueMat = new Mat();

    Paint red = new Paint();
    Paint blue = new Paint();
    Paint yellow = new Paint();

    Scalar blueLower = new Scalar(80,55,30);
    Scalar blueHigher = new Scalar(170,255,255);

    Scalar redLower = new Scalar(0,80,0);
    Scalar redHigher = new Scalar(15,255,255);

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

    Point[] vertices;

    Point centerPoint = new Point();
    Point xLeft = new Point();
    Point xRight = new Point();

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
    }

    @Override
    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
        continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
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

        for (int i = 0; i < blueContours.size(); i++){
            Rect rect = boundingRect(blueContours.get(i));
            if (rect.area() > 1000){
                blueRects.add(rect);
            }
        }

        for (int i = 0; i < redContours.size(); i++){
            Rect rect = boundingRect(redContours.get(i));
            if (rect.area() > 2000){
                redRects.add(rect);
                sortedRedContours.add(redContours.get(i));
            }
        }

        for (int i = 0; i < yellowContours.size(); i++){
            Rect rect = boundingRect(yellowContours.get(i));
            if (rect.area() > 1000){
                yellowRects.add(rect);
            }
        }

        for (MatOfPoint contour : sortedRedContours) {

            telemetry.addData("contour.toArray().length", contour.toArray().length);
            telemetry.update();

            List<Point> contourPointsX = Arrays.asList(contour.toArray());
            ArrayList<Point> farXPoints = new ArrayList<>();
            List<Point> contourPointsLowerX = Arrays.asList(contour.toArray());
            List<Point> contourPointsUpperX = Arrays.asList(contour.toArray());

            xRight = Collections.max(contourPointsUpperX, Comparator.comparingDouble(p -> p.x));

            for (int i = 0; i < contourPointsX.size(); i++){
                if (Math.abs(xRight.x - contourPointsX.get(i).x) < 3){
                    farXPoints.add(contourPointsX.get(i));
                }
            }

            farXPoints.sort(Comparator.comparingDouble(p -> p.y));
            centerPoint = farXPoints.subList(0, 1).get(0);

            contourPointsLowerX.sort(Comparator.comparingDouble(p -> p.x));
            xLeft = contourPointsLowerX.subList(0, 1).get(0);

//            double hypot = (Math.abs(Math.hypot(xRight.x - xLeft.x, xRight.y - xLeft.y)))/2;
            double deltaX = (xRight.x - xLeft.x)/2;
            double deltaY = (xRight.y - xLeft.y)/2;

            centerPoint = new Point(xLeft.x + deltaX, xLeft.y + deltaY);

            telemetry.addData("yTop", centerPoint);
            telemetry.update();

        }

//        redMat.copyTo(frame);

        System.out.println("width" + frame.width());
        System.out.println("height" + frame.height());

        blueContours.clear();
        yellowContours.clear();
        redContours.clear();
        sortedRedContours.clear();

        Bitmap b = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(frame, b);
        lastFrame.set(b);
        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        setUpPaints(scaleCanvasDensity);

        if (!blueRects.isEmpty()){
            for (int i = 0; i < blueRects.size(); i++){
                canvas.drawRect(makeGraphicsRect(blueRects.get(i), scaleBmpPxToCanvasPx), blue);
            }
        }

        if (!yellowRects.isEmpty()){
            for (int i = 0; i < yellowRects.size(); i++){
                canvas.drawRect(makeGraphicsRect(yellowRects.get(i), scaleBmpPxToCanvasPx), yellow);
            }
        }

        if (!redRects.isEmpty()){
            for (int i = 0; i < redRects.size(); i++){
                canvas.drawRect(makeGraphicsRect(redRects.get(i), scaleBmpPxToCanvasPx), red);
            }
        }

//        canvas.drawLine((float) topLeft.x * scaleBmpPxToCanvasPx, (float) topLeft.y * scaleBmpPxToCanvasPx, (float) topRight.x * scaleBmpPxToCanvasPx, (float) topRight.y * scaleBmpPxToCanvasPx, blue); // Top edge
//        canvas.drawLine((float) topLeft.x * scaleBmpPxToCanvasPx, (float) topLeft.y * scaleBmpPxToCanvasPx, (float) bottomLeft.x * scaleBmpPxToCanvasPx, (float) bottomLeft.y * scaleBmpPxToCanvasPx, blue); // Left edge
//        canvas.drawLine((float) topRight.x * scaleBmpPxToCanvasPx, (float) topRight.y * scaleBmpPxToCanvasPx, (float) bottomRight.x * scaleBmpPxToCanvasPx, (float) bottomRight.y * scaleBmpPxToCanvasPx, blue); // Right edge
//        canvas.drawLine((float) bottomLeft.x * scaleBmpPxToCanvasPx, (float) bottomLeft.y * scaleBmpPxToCanvasPx, (float) bottomRight.x * scaleBmpPxToCanvasPx, (float) bottomRight.y * scaleBmpPxToCanvasPx, blue); // Bottom edge

//        for(int i = 0; i < 4; i++){
//            canvas.drawLine((float) vertices[i].x * scaleBmpPxToCanvasPx, (float) vertices[i].y * scaleBmpPxToCanvasPx, (float) vertices[(i +1) %4].x * scaleBmpPxToCanvasPx, (float) vertices[(i +1) %4].y * scaleBmpPxToCanvasPx, red);
//        }

        canvas.drawCircle((float) centerPoint.x+3, (float) centerPoint.y+3, 6, blue); // Draw a small circle at top-left
        canvas.drawCircle((float) xLeft.x+3, (float) xLeft.y+3, 6, blue); // Draw a small circle at top-right
        canvas.drawCircle((float) xRight.x+3, (float) xRight.y+3, 6, blue); // Draw a small circle at bottom-left
//        canvas.drawCircle((float) bottomRight.x, (float) bottomRight.y, 5, blue);
//
//        System.out.println(topLeft.x);
//        System.out.println(topLeft.y);
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
