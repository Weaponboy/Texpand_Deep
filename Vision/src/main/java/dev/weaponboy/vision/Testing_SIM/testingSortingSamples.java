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

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;

public class testingSortingSamples implements VisionProcessor {

    Mat redMat = new Mat();
    Paint red = new Paint();

    Mat yellowMat = new Mat();
    Paint yellow = new Paint();

    public Scalar redLower = new Scalar(0,120,229);
    public Scalar redHigher = new Scalar(15,255,255);

    public Scalar yellowLower = new Scalar(10,120,240);
    public Scalar yellowHigher = new Scalar(25,255,255);

    ArrayList<MatOfPoint> redContours = new ArrayList<>();
    ArrayList<MatOfPoint> singleSamplesRed = new ArrayList<>();
    ArrayList<Rect> mutiSamplesRed = new ArrayList<>();
    Mat redHierarchy = new Mat();
    Point[] redVertices = new Point[4];

    ArrayList<MatOfPoint> yellowContours = new ArrayList<>();
    ArrayList<MatOfPoint> sortedYellowContours = new ArrayList<>();
    Mat yellowHierarchy = new Mat();
    Point[] yellowVertices = new Point[4];

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame, redMat, COLOR_RGB2HSV);

        inRange(redMat, redLower, redHigher, redMat);

        erode(redMat, redMat, new Mat(5, 5, CV_8U));

        dilate(redMat, redMat, new Mat(5, 5, CV_8U));

        findContours(redMat, redContours, redHierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

        for (int i = 0; i < redContours.size(); i++){
            Rect rect = boundingRect(redContours.get(i));
            if (rect.area() > 10000 && rect.area() < 35000){
                singleSamplesRed.add(redContours.get(i));
            } else if (rect.area() > 60000) {
                mutiSamplesRed.add(rect);
            }
        }

        for (MatOfPoint contour : singleSamplesRed) {
            RotatedRect box = Imgproc.minAreaRect(new MatOfPoint2f(contour.toArray()));
            box.points(redVertices);

//            MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
//            MatOfPoint2f approxCurve = new MatOfPoint2f();
//
//            Imgproc.approxPolyDP(contour2f, approxCurve, Imgproc.arcLength(contour2f, true) * 0.02, true);
//
//            if (approxCurve.total() >= 4) {
//                contourPoints.addAll(Arrays.asList(approxCurve.toArray()));
//            }


        }

//        rectangles.clear();
//
//        rectangles = (ArrayList<RotatedRect>) RectangleDetector.detectRectangles(contourPoints);
//
//        telemetry.addData("rectangles", rectangles.size());
//        telemetry.update();

        System.out.println("width" + frame.width());
        System.out.println("height" + frame.height());

        redContours.clear();
        singleSamplesRed.clear();

        redMat.copyTo(frame);
        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        red.setColor(Color.RED);
        red.setStyle(Paint.Style.STROKE);
        red.setStrokeWidth(scaleCanvasDensity * 4);

//        for (Point point : contourPoints){
//            canvas.drawCircle((float) point.x+3, (float) point.y+3, 6, red);
//        }

//        if (redVertices[0].x > 0){
//            for (int j = 0; j < 4; j++) {
//                canvas.drawLine((float) redVertices[j].x , (float) redVertices[j].y , (float) redVertices[(j + 1) % 4].x, (float) redVertices[(j + 1) % 4].y , red);
//            }
//        }


//        for (RotatedRect rect : rectangles) {
//            Point[] rectPoints = new Point[4];
//            rect.points(rectPoints);
//        }
    }


}
