package dev.weaponboy.vision.SamplePipelines;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import java.util.concurrent.atomic.AtomicReference;

import dev.weaponboy.vision.cameracalibration;

public class PerspectiveTransformPipeline implements VisionProcessor, CameraStreamSource {

    private final AtomicReference<Bitmap> lastFrame =
            new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

    cameracalibration defaultCalibration = new cameracalibration();

    // Define the size of the output (top-down) view
    private static final int WIDTH = 400;
    private static final int HEIGHT = 300;
    private Mat outputMat = new Mat();

    Rect ROI = new Rect(450, 250, 400, 300);

    @Override
    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
        continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {

        Mat frameSub = frame.submat(ROI);

//        frameSub = defaultCalibration.undistortImage(frameSub);

        Mat undistortedFrame = new Mat();

        Calib3d.undistort(frameSub, undistortedFrame, defaultCalibration.cameraMatrix, defaultCalibration.distCoeffs);

        Point[] srcPoints = new Point[]{
                new Point(76, 0),  // Top-left corner in camera view
                new Point(300, 0),  // Top-right corner
                new Point(400, 300),  // Bottom-right corner
                new Point(0, 300)   // Bottom-left corner
        };

        // Define the destination points for the top-down view
        Point[] dstPoints = new Point[]{
                new Point(0, 0),            // Top-left corner of the top-down view
                new Point(WIDTH, 0),        // Top-right corner
                new Point(WIDTH, HEIGHT),   // Bottom-right corner
                new Point(0, HEIGHT)        // Bottom-left corner
        };

        // Convert arrays to MatOfPoint2f
        MatOfPoint2f srcMat = new MatOfPoint2f(srcPoints);
        MatOfPoint2f dstMat = new MatOfPoint2f(dstPoints);

        // Calculate the homography matrix
        Mat homographyMatrix = Calib3d.findHomography(srcMat, dstMat);

        // Apply the perspective warp
        Imgproc.warpPerspective(undistortedFrame, outputMat, homographyMatrix, ROI.size());

        outputMat.copyTo(undistortedFrame);

        Bitmap b = Bitmap.createBitmap(undistortedFrame.width(), undistortedFrame.height(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(undistortedFrame, b);
        lastFrame.set(b);

        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
}