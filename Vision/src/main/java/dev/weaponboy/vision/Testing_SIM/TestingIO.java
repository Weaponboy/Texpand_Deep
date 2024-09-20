package dev.weaponboy.vision.Testing_SIM;

import static org.opencv.core.Core.inRange;
import static org.opencv.core.CvType.CV_32S;
import static org.opencv.core.CvType.CV_8U;
import static org.opencv.imgproc.Imgproc.CHAIN_APPROX_SIMPLE;
import static org.opencv.imgproc.Imgproc.COLOR_RGB2HSV;
import static org.opencv.imgproc.Imgproc.RETR_TREE;
import static org.opencv.imgproc.Imgproc.boundingRect;
import static org.opencv.imgproc.Imgproc.dilate;
import static org.opencv.imgproc.Imgproc.erode;
import static org.opencv.imgproc.Imgproc.findContours;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class TestingIO extends OpenCvPipeline {

    Mat hsvMat = new Mat();
    Mat thresholdMat = new Mat();

    public Scalar redLower = new Scalar(0, 12, 147);
    public Scalar redHigher = new Scalar(15, 255, 255);

    @Override
    public Mat processFrame(Mat input) {

        // Convert the image to HSV color space
        Imgproc.cvtColor(input, hsvMat, COLOR_RGB2HSV);

        // Threshold the HSV image to get only red colors
        inRange(hsvMat, redLower, redHigher, thresholdMat);

        // Erode and dilate to remove noise
        erode(thresholdMat, thresholdMat, new Mat(5, 5, CV_8U));
        dilate(thresholdMat, thresholdMat, new Mat(5, 5, CV_8U));

        // Find contours
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        findContours(thresholdMat, contours, hierarchy, Imgproc.RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        // Process each contour
        for (MatOfPoint contour : contours) {

            // Filter contours based on area
            Rect rect = boundingRect(contour);
            if (rect.area() < 3000) { // Adjust area threshold as needed
                continue;
            }

            // Approximate the contour to a polygon
            MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
            double epsilon = 0.02 * Imgproc.arcLength(contour2f, true);
            MatOfPoint2f approxCurve = new MatOfPoint2f();
            Imgproc.approxPolyDP(contour2f, approxCurve, epsilon, true);

            // Convert back to MatOfPoint
            MatOfPoint approxContour = new MatOfPoint();
            approxCurve.convertTo(approxContour, CvType.CV_32S);

            Point[] polyPoints = approxContour.toArray();

            if (polyPoints.length < 4) {
                continue; // Not enough points to form a quadrilateral
            }

            // Find the top corner point (smallest y)
            Point topCornerPoint = polyPoints[0];
            int topCornerIndex = 0;
            for (int i = 1; i < polyPoints.length; i++) {
                if (polyPoints[i].y < topCornerPoint.y) {
                    topCornerPoint = polyPoints[i];
                    topCornerIndex = i;
                }
            }

            // Find the adjacent vertices
            int prevIndex = (topCornerIndex - 1 + polyPoints.length) % polyPoints.length;
            int nextIndex = (topCornerIndex + 1) % polyPoints.length;

            Point prevPoint = polyPoints[prevIndex];
            Point nextPoint = polyPoints[nextIndex];

            // Calculate the lengths of the edges adjacent to the top corner
            double length1 = getDistance(topCornerPoint, prevPoint);
            double length2 = getDistance(topCornerPoint, nextPoint);

            // Determine which length is longer
            double longerLength = Math.max(length1, length2);
            double shorterLength = Math.min(length1, length2);

            // Calculate the ratio of the side lengths
            double sideRatio = longerLength / shorterLength;

            // Check if the ratio falls within the acceptable range
            if (sideRatio < 1.4 || sideRatio > 2.2) {
                continue; // Skip this contour if the ratio is not acceptable
            }

            // Mark the top corner point
            Imgproc.circle(input, topCornerPoint, 5, new Scalar(0, 255, 0), -1);

            // Draw lines from top corner to adjacent points in blue
            Imgproc.line(input, topCornerPoint, prevPoint, new Scalar(255, 0, 0), 2);
            Imgproc.line(input, topCornerPoint, nextPoint, new Scalar(255, 0, 0), 2);

            // Determine corner orientation based on positions of adjacent points
            String cornerOrientation = determineCornerOrientation(topCornerPoint, prevPoint, nextPoint);

            // Display the corner orientation
            Imgproc.putText(input, cornerOrientation, new Point(topCornerPoint.x + 10, topCornerPoint.y - 10),
                    Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
        }

        return input; // Return the original image with drawings
    }

    public String determineCornerOrientation(Point topCorner, Point prevPoint, Point nextPoint) {
        // Calculate the average X positions of the adjacent points
        double avgX = (prevPoint.x + nextPoint.x) / 2;

        if (avgX < topCorner.x) {
            return "Top-Right";
        } else {
            return "Top-Left";
        }
    }

    public double getDistance(Point p1, Point p2) {
        double dx = p1.x - p2.x;
        double dy = p1.y - p2.y;
        return Math.sqrt(dx * dx + dy * dy);
    }
}
