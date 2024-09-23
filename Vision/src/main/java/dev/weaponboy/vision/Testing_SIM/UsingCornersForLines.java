package dev.weaponboy.vision.Testing_SIM;

import static org.opencv.core.Core.inRange;
import static org.opencv.core.CvType.CV_8U;
import static org.opencv.imgproc.Imgproc.CHAIN_APPROX_SIMPLE;
import static org.opencv.imgproc.Imgproc.COLOR_RGB2HSV;
import static org.opencv.imgproc.Imgproc.boundingRect;
import static org.opencv.imgproc.Imgproc.dilate;
import static org.opencv.imgproc.Imgproc.erode;
import static org.opencv.imgproc.Imgproc.findContours;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;

public class UsingCornersForLines extends OpenCvPipeline {

    Mat redMat = new Mat();

    public Scalar redLower = new Scalar(0,95,40);
    public Scalar redHigher = new Scalar(15,255,255);

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

    @Override
    public Mat processFrame(Mat input) {

        Imgproc.cvtColor(input, redMat, COLOR_RGB2HSV);

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

        Imgproc.putText(input, String.valueOf(redContours.size()), new Point(200, 290), 2, 1, new Scalar(0, 255, 0), 4, 2, false);

        for (MatOfPoint contour : contours) {
            Imgproc.drawContours(input, Arrays.asList(contour), -1, new Scalar(0, 255, 0), 2);
        }
//
//        int lookAhead = 10; // Number of points to look ahead along the contour for angle change detection
//        double searchRadius = 20.0;  // Define a search radius around the highest point
//        double minAngle = 100.0;     // Minimum angle between two nearby points
//        double maxAngle = 160.0;     // Maximum angle between two nearby points
//        double angleTolerance = 30.0; // Tolerance for considering points along the same angle
//        double cornerAngleThreshold = 100; // Minimum angle change that signifies a corner
//
//// PART 1: Verify if the highest point is a valid corner
//        for (MatOfPoint contour : redContours) {
//
//            // Get points from contour
//            List<Point> contourPointsY = Arrays.asList(contour.toArray());
//
//            // Sort by y-coordinate to find the top point (highest 'y')
//            contourPointsY.sort(Comparator.comparingDouble(p -> p.y));
//            Point highestPoint = contourPointsY.get(0); // The highest point
//
//            // Add the top point to the list for visualization later
//            topY.add(highestPoint);
//
//            // Search for nearby points within a radius from the highest point
//            List<Point> nearbyPoints = new ArrayList<>();
//            for (Point p : contourPointsY) {
//                if (Math.hypot(p.x - highestPoint.x, p.y - highestPoint.y) <= searchRadius) {
//                    nearbyPoints.add(p); // Collect points within the search radius
//                }
//            }
//
//            // Ensure we have enough nearby points to check for a corner
//            if (nearbyPoints.size() < 2) continue; // Not enough nearby points to verify a corner
//
//            // Find the two farthest nearby points
//            Point nearbyPoint1 = null;
//            Point nearbyPoint2 = null;
//            double maxDistance1 = 0;
//            double maxDistance2 = 0;
//
//            for (Point p : nearbyPoints) {
//                double distance = Math.hypot(p.x - highestPoint.x, p.y - highestPoint.y);
//
//                if (distance > maxDistance1) {
//                    maxDistance2 = maxDistance1;
//                    nearbyPoint2 = nearbyPoint1;
//
//                    maxDistance1 = distance;
//                    nearbyPoint1 = p;
//                } else if (distance > maxDistance2) {
//                    maxDistance2 = distance;
//                    nearbyPoint2 = p;
//                }
//            }
//
//            // Verify if the highest point is a corner by checking the angle between nearby points
//            if (nearbyPoint1 != null && nearbyPoint2 != null) {
//                double angle1 = Math.toDegrees(Math.atan2(nearbyPoint1.y - highestPoint.y, nearbyPoint1.x - highestPoint.x));
//                double angle2 = Math.toDegrees(Math.atan2(nearbyPoint2.y - highestPoint.y, nearbyPoint2.x - highestPoint.x));
//
//                double angleDifference = Math.abs(angle1 - angle2);
//                if (angleDifference > 180) {
//                    angleDifference = 360 - angleDifference;
//                }
//
//                if (angleDifference >= minAngle && angleDifference <= maxAngle) {
//                    // The highest point is considered a corner since the angle difference is valid
//                    Imgproc.circle(input, highestPoint, 4, new Scalar(0, 0, 255), -1); // Draw the corner point
//
//                    // PART 2: Search for the closest corner along each angle
//
//                    // Find the closest corner along angle1 by looking ahead 20 points
//                    Point closestCornerAlongAngle1 = null;
//                    double lastAngle1 = angle1;
//
//                    for (int i = 0; i < contourPointsY.size() - lookAhead; i++) {
//                        Point p = contourPointsY.get(i);
//                        Point nextPoint = contourPointsY.get(i + lookAhead);
//
//                        double currentAngle = Math.toDegrees(Math.atan2(p.y - highestPoint.y, p.x - highestPoint.x));
//                        double nextAngle = Math.toDegrees(Math.atan2(nextPoint.y - highestPoint.y, nextPoint.x - highestPoint.x));
//                        double angleDiff = Math.abs(nextAngle - lastAngle1);
//
//                        if (angleDiff > 180) {
//                            angleDiff = 360 - angleDiff;
//                        }
//
//                        if (Math.abs(currentAngle - angle1) <= angleTolerance && angleDiff > cornerAngleThreshold) {
//                            closestCornerAlongAngle1 = p; // Found a corner along this angle
//                            break; // Stop searching further
//                        }
//                    }
//
//                    // Find the closest corner along angle2 by looking ahead 20 points
//                    Point closestCornerAlongAngle2 = null;
//                    double lastAngle2 = angle2;
//
//                    for (int i = 0; i < contourPointsY.size() - lookAhead; i++) {
//                        Point p = contourPointsY.get(i);
//                        Point nextPoint = contourPointsY.get(i + lookAhead);
//
//                        double currentAngle = Math.toDegrees(Math.atan2(p.y - highestPoint.y, p.x - highestPoint.x));
//                        double nextAngle = Math.toDegrees(Math.atan2(nextPoint.y - highestPoint.y, nextPoint.x - highestPoint.x));
//                        double angleDiff = Math.abs(nextAngle - lastAngle2);
//
//                        if (angleDiff > 180) {
//                            angleDiff = 360 - angleDiff;
//                        }
//
//                        if (Math.abs(currentAngle - angle2) <= angleTolerance && angleDiff > cornerAngleThreshold) {
//                            closestCornerAlongAngle2 = p; // Found a corner along this angle
//                            break; // Stop searching further
//                        }
//                    }
//
//                    // Draw lines from the corner to the closest corners along each angle
//                    if (closestCornerAlongAngle1 != null) {
//                        Imgproc.line(input, highestPoint, closestCornerAlongAngle1, new Scalar(0, 0, 255), 2); // Green line for angle1
//                    }
//                    if (closestCornerAlongAngle2 != null) {
//                        Imgproc.line(input, highestPoint, closestCornerAlongAngle2, new Scalar(0, 0, 255), 2); // Green line for angle2
//                    }
//                }
//            }
//        }
//

        double minAngle = 20.0;   // Minimum angle to consider a point as a corner
        double maxAngle = 240.0;  // Maximum angle to consider a point as a corner
        int lookAhead = 10;       // Number of points to look ahead/backward for angle checking
        double minDistance = 40.0; // Minimum distance between corners to avoid duplicates

        List<Point> corners = new ArrayList<>();  // List to store detected corners

// Part 1: Detect corners based on angle change
        for (MatOfPoint contour : redContours) {
            List<Point> contourPoints = Arrays.asList(contour.toArray());

            // Loop through each point in the contour
            for (int i = lookAhead; i < contourPoints.size() - lookAhead; i++) {
                Point previousPoint = contourPoints.get(i - lookAhead);  // Look ahead/backward points
                Point currentPoint = contourPoints.get(i);
                Point nextPoint = contourPoints.get(i + lookAhead);

                // Calculate the angle between the previous, current, and next points
                double angle1 = Math.toDegrees(Math.atan2(currentPoint.y - previousPoint.y, currentPoint.x - previousPoint.x));
                double angle2 = Math.toDegrees(Math.atan2(nextPoint.y - currentPoint.y, nextPoint.x - currentPoint.x));

                // Calculate the difference in angles
                double angleDifference = Math.abs(angle1 - angle2);
                if (angleDifference > 180) {
                    angleDifference = 360 - angleDifference;  // Ensure angleDifference is always between 0 and 180
                }

                // Check if the angle difference qualifies the point as a corner
                if (angleDifference >= minAngle && angleDifference <= maxAngle) {
                    corners.add(currentPoint);  // Add the current point to the list of corners
                }
            }
        }

// Part 2: Remove duplicate corners that are too close to each other
        List<Point> uniqueCorners = new ArrayList<>();  // List to store unique corners

        for (Point corner : corners) {
            boolean isDuplicate = false;

            // Check distance with all previously added unique corners
            for (Point uniqueCorner : uniqueCorners) {
                double distance = Math.hypot(corner.x - uniqueCorner.x, corner.y - uniqueCorner.y);
                if (distance < minDistance) {
                    isDuplicate = true;  // Mark as duplicate if within minDistance
                    break;
                }
            }

            // If not a duplicate, add to unique corners
            if (!isDuplicate) {
                uniqueCorners.add(corner);
            }
        }

// Part 3: Visualize the unique corners and draw lines (if needed)
        for (Point corner : uniqueCorners) {
            Imgproc.circle(input, corner, 4, new Scalar(0, 0, 255), -1);  // Draw red circles for corners
        }
//
//        for (int i = 0; i < uniqueCorners.size() - 1; i++) {
//            Imgproc.line(input, uniqueCorners.get(i), uniqueCorners.get(i + 1), new Scalar(0, 255, 0), 2);  // Draw green lines between unique corners
//        }


        redContours.clear();
        topY.clear();

        return input;
    }

    public Point createRotatedRectFromTopLeft(Point topLeft, Size size, double angle) {
        double radians = Math.toRadians(angle);

        double centerX = topLeft.x + (size.width / 2) * Math.cos(radians) - (size.height / 2) * Math.sin(radians);
        double centerY = topLeft.y + (size.width / 2) * Math.sin(radians) + (size.height / 2) * Math.cos(radians);
        Point center = new Point(centerX, centerY);

        return center;
    }

    public void findSides(Mat input, Point highestPoint, double searchRadius, List<Point> contourPointsY){
        List<Point> nearbyPoints = new ArrayList<>();
        for (Point p : contourPointsY) {
            if (Math.hypot(p.x - highestPoint.x, p.y - highestPoint.y) <= searchRadius) {
                nearbyPoints.add(p); // Collect points within the search radius
            }
        }

        Imgproc.circle(input, highestPoint, 4, new Scalar(0, 0, 255), -1); // Red circle

        // Find the two farthest points from the highest point
        Point farthestPoint1 = null;
        Point farthestPoint2 = null;
        double maxDistance1 = 0;
        double maxDistance2 = 0;

        for (Point p : nearbyPoints) {
            double distance = Math.hypot(p.x - highestPoint.x, p.y - highestPoint.y);

            if (distance > maxDistance1) {
                // Shift the current farthest point to second farthest
                maxDistance2 = maxDistance1;
                farthestPoint2 = farthestPoint1;

                // Set the new farthest point
                maxDistance1 = distance;
                farthestPoint1 = p;
            } else if (distance > maxDistance2) {
                maxDistance2 = distance;
                farthestPoint2 = p;
            }
        }

        Imgproc.circle(input, highestPoint, 4, new Scalar(0, 0, 255), -1); // Red circle

        // Draw lines from the highest point to the two farthest points
        if (farthestPoint1 != null && farthestPoint2 != null) {
            Imgproc.line(input, highestPoint, farthestPoint1, new Scalar(0, 255, 0), 2); // Green line
            Imgproc.line(input, highestPoint, farthestPoint2, new Scalar(0, 255, 0), 2); // Green line

            // Draw the detected corner point
            Imgproc.circle(input, highestPoint, 4, new Scalar(0, 0, 255), -1); // Red circle
        }
    }

    public double getHypot(Point one, Point two){
        double deltaX = Math.abs(one.x - two.x);
        double deltaY = Math.abs(one.y - two.y);
        return Math.hypot(deltaY, deltaX);
    }


}
