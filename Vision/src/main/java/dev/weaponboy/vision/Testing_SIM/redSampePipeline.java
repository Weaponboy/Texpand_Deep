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

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

public class redSampePipeline extends OpenCvPipeline {

    Mat redMat = new Mat();

    public Scalar redLower = new Scalar(9, 40, 160);
    public Scalar redHigher = new Scalar(38, 255, 255);

    ArrayList<MatOfPoint> redContours = new ArrayList<>();
    ArrayList<MatOfPoint> sortedRedContours = new ArrayList<>();
    Mat redHierarchy = new Mat();

    @Override
    public Mat processFrame(Mat input) {

        Imgproc.cvtColor(input, redMat, COLOR_RGB2HSV);

        inRange(redMat, redLower, redHigher, redMat);

        erode(redMat, redMat, new Mat(5, 5, CV_8U));

        dilate(redMat, redMat, new Mat(5, 5, CV_8U));

        Mat redHierarchy = new Mat();
        Imgproc.findContours(redMat, redContours, redHierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        for (MatOfPoint contour : redContours) {
            Imgproc.drawContours(input, Arrays.asList(contour), -1, new Scalar(0, 255, 0), 2);
        }

        for (int i = 0; i < redContours.size(); i++) {
            Rect rect = boundingRect(redContours.get(i));
            if (rect.area() > 30000 && rect.area() < 200000) {
                sortedRedContours.add(redContours.get(i));
            }
        }

        for (MatOfPoint contour : sortedRedContours) {
//
//            // Approximate contour to polygon (rectangular approximation)
//            MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
//            MatOfPoint2f approx = new MatOfPoint2f();
//            Imgproc.approxPolyDP(contour2f, approx, Imgproc.arcLength(contour2f, true) * 0.04, true);
//
//            // Check if we have a rectangle (4 vertices)
//            if (approx.total() == 4) {
//                Point[] vertices = approx.toArray();
//                List<Line> sides = new ArrayList<>();
//
//                // Find long and short sides
//                for (int i = 0; i < 4; i++) {
//                    Point p1 = vertices[i];
//                    Point p2 = vertices[(i + 1) % 4]; // Next point (wrap around)
//                    sides.add(new Line(p1, p2));
//                }
//
//                // Sort sides by length to differentiate long and short sides
//                sides.sort((l1, l2) -> Double.compare(l1.length(), l2.length()));
//                Line longSide1 = sides.get(3);
//                Line longSide2 = sides.get(2); // Two longest sides
//
//                // Try to move the long sides inward
//                Line inwardLongSide1 = moveLineInward(longSide1, 10.0); // Move long side inward by 10 units
//                Line inwardLongSide2 = moveLineInward(longSide2, 10.0);
//
//                // If not enough space, move the short sides inward
//                Line inwardShortSide1 = moveLineInward(sides.get(0), 10.0);
//                Line inwardShortSide2 = moveLineInward(sides.get(1), 10.0);
//
//                // Find intersection points between the inward long/short sides
//                Point intersectionLong = getLineIntersection(inwardLongSide1, inwardLongSide2);
//                Point intersectionShort = getLineIntersection(inwardShortSide1, inwardShortSide2);
//
//                // Decide which intersection to use (based on fitting logic or priority)
//                Point targetPoint = (intersectionLong != null) ? intersectionLong : intersectionShort;
//
//                // Draw the target point for visualization
//                if (targetPoint != null) {
//                    Imgproc.circle(input, targetPoint, 5, new Scalar(255, 0, 0), -1); // Red dot for target
//                }
//
//            }
//
//            Imgproc.putText(input, String.valueOf(10000), new Point(20, 40), 2, 1, new Scalar(0, 255, 0), 4, 2, false);
//
//

            // Find a 90-degree corner with a tolerance of ±10 degrees
            // Detect target point by moving 10 pixels inward
            // Detect target point by moving 10 pixels and 5 pixels inward
            double inwardDistance1 = 50;
            double inwardDistance = 200;
//            List<Point> inwardPoints = detectTargetPoints(contour.toList(), inwardDistance);
//
//            // Print out the inward points
//            for (Point p : inwardPoints) {
//                System.out.println("Inward Point: " + p);
//                Imgproc.circle(input, p, 5, new Scalar(255, 0, 0), -1); // Red dot for target
//            }

            // Distance to move inward
//            double inwardDistance = 10.0;

            // List to store inward points
            List<Point> inwardPoints = new ArrayList<>();

            MatOfPoint2f contourMat = new MatOfPoint2f(contour.toArray());

            // Calculate inward points along normals
            Point[] contourPoints = contour.toArray();

            for (int i = 0; i < contourPoints.length; i++) {
                // Get current point and previous/next point to calculate normal
                Point p1 = contourPoints[(i - 1 + contourPoints.length) % contourPoints.length]; // previous point (wrap around)
                Point p2 = contourPoints[i]; // current point
                Point p3 = contourPoints[(i + 1) % contourPoints.length]; // next point

                // Calculate the direction vector between previous and next points
                double dx = p3.x - p1.x;
                double dy = p3.y - p1.y;
                double magnitude = Math.sqrt(dx * dx + dy * dy);

                // Normalize and calculate normal (rotating 90 degrees)
                // Invert the normal to move inward (negating it)
                Point normal = new Point(dy / magnitude, -dx / magnitude); // Flip to ensure inward direction

                // Calculate inward point by moving along the normal (inward distance is applied)
                Point inwardPoint = new Point(p2.x + inwardDistance * normal.x, p2.y + inwardDistance * normal.y);

                // Calculate the distance of the inward point from the contour
                double distanceToEdge = Imgproc.pointPolygonTest(contourMat, inwardPoint, true);

                // Only add the point if it's more than 50 pixels away from the edge
                if (distanceToEdge > 50) {
                    inwardPoints.add(inwardPoint);
                }
            }

            for (int i = 0; i < contourPoints.length; i++) {
                // Get current point and previous/next point to calculate normal
                Point p1 = contourPoints[(i - 1 + contourPoints.length) % contourPoints.length]; // previous point (wrap around)
                Point p2 = contourPoints[i]; // current point
                Point p3 = contourPoints[(i + 1) % contourPoints.length]; // next point

                // Calculate the direction vector between previous and next points
                double dx = p3.x - p1.x;
                double dy = p3.y - p1.y;
                double magnitude = Math.sqrt(dx * dx + dy * dy);

                // Normalize and calculate normal (rotating 90 degrees)
                // Invert the normal to move inward (negating it)
                Point normal = new Point(dy / magnitude, -dx / magnitude); // Flip to ensure inward direction

                // Calculate inward point by moving along the normal (inward distance is applied)
                Point inwardPoint = new Point(p2.x + inwardDistance1 * normal.x, p2.y + inwardDistance1 * normal.y);

                // Calculate the distance of the inward point from the contour
                double distanceToEdge = Imgproc.pointPolygonTest(contourMat, inwardPoint, true);

                // Only add the point if it's more than 50 pixels away from the edge
                if (distanceToEdge > 50) {
                    inwardPoints.add(inwardPoint);
                }
            }


            Point TargetPoint = findCentroid(inwardPoints);

            Imgproc.circle(input, TargetPoint, 5, new Scalar(0, 255, 0), -1); // Draw green circles for inward points

            // Draw inward points on a new image (for visualization)
//            Mat resultImage = new Mat(image.size(), CvType.CV_8UC3, new Scalar(255, 255, 255)); // Create white canvas
            for (Point p : inwardPoints) {
                Imgproc.circle(input, p, 3, new Scalar(255, 0, 0), -1); // Draw green circles for inward points
            }

//            System.out.println("Target point for the claw: " + targetPoint);
//
//            Imgproc.putText(input, String.valueOf(targetPoint), new Point(20, 80), 2, 1, new Scalar(0, 255, 0), 4, 2, false);
//            Imgproc.circle(input, targetPoint, 5, new Scalar(0, 255, 0), -1); // Red dot for target

//
//            if (!corners.isEmpty()) {
//                System.out.println("90-degree corners found at:");
//                for (Point corner : corners) {
//                    System.out.println(corner);
//
//                }
//            } else {
//                System.out.println("No 90-degree corners found.");
//            }
        }

        redContours.clear();
        sortedRedContours.clear();

        return input;
    }

    public static Point findCentroid(List<Point> points) {
        double sumX = 0;
        double sumY = 0;
        for (Point p : points) {
            sumX += p.x;
            sumY += p.y;
        }
        return new Point(sumX / points.size(), sumY / points.size());
    }


    public static List<Point> findInwardPointsForRectangle(List<Point> contour, double inwardDistance) {
        List<Point> inwardPoints = new ArrayList<>();
        int size = contour.size();

        for (int i = 0; i < size; i++) {
            // Get the current point and the next point (to form a line segment)
            Point p1 = contour.get(i);                      // Current point
            Point p2 = contour.get((i + 1) % size);         // Next point (with wrap-around)

            // Calculate the direction vector of the side (p2 - p1)
            Point sideVector = new Point(p2.x - p1.x, p2.y - p1.y);

            // Calculate the normal vector (perpendicular to the side)
            Point inwardNormal = new Point(-sideVector.y, sideVector.x);
            double length = Math.sqrt(inwardNormal.x * inwardNormal.x + inwardNormal.y * inwardNormal.y);

            // Normalize the normal vector
            inwardNormal.x /= length;
            inwardNormal.y /= length;

            // Move both p1 and p2 inward by the specified distance along the normal
            Point inwardP1 = new Point(
                    p1.x + inwardDistance * inwardNormal.x,
                    p1.y + inwardDistance * inwardNormal.y
            );
            Point inwardP2 = new Point(
                    p2.x + inwardDistance * inwardNormal.x,
                    p2.y + inwardDistance * inwardNormal.y
            );

            // Ensure the inward points remain inside the contour
            if (isPointInContour(inwardP1, contour)) {
                inwardPoints.add(inwardP1);
            }
            if (isPointInContour(inwardP2, contour)) {
                inwardPoints.add(inwardP2);
            }
        }

        return inwardPoints;
    }

    /**
     * Checks if a point is inside the contour using point polygon test.
     *
     * @param point The point to check.
     * @param contour The contour represented as a list of points.
     * @return True if the point is inside the contour, false otherwise.
     */
    private static boolean isPointInContour(Point point, List<Point> contour) {
        // Convert the contour to an array of Point2f for OpenCV
        Point[] contourArray = contour.toArray(new Point[0]);
        return Imgproc.pointPolygonTest(new MatOfPoint2f(contourArray), point, false) >= 0;
    }

    /**
     * Detects the target points based on inward points from all sides of a rectangle.
     *
     * @param contour The list of points representing the rectangular contour.
     * @param inwardDistance The distance to move inward from each side.
     * @return The list of inward points that remain inside the contour.
     */
    public static List<Point> detectTargetPoints(List<Point> contour, double inwardDistance) {
        // Find inward points for the rectangle
        return findInwardPointsForRectangle(contour, inwardDistance);
    }


}

