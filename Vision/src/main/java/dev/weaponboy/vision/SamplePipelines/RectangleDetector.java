package dev.weaponboy.vision.SamplePipelines;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import java.util.*;
import java.util.stream.Collectors;

public class RectangleDetector {

    public static boolean isRectangle(Point p1, Point p2, Point p3, Point p4) {
        double d1 = distance(p1, p2);
        double d2 = distance(p2, p3);
        double d3 = distance(p3, p4);
        double d4 = distance(p4, p1);
        
        boolean isValidSides = Math.abs(d1 - d3) < 1e-2 && Math.abs(d2 - d4) < 1e-2;

        // Calculate the angles between adjacent sides (they should be close to 90 degrees)
        double angle1 = getAngle(p1, p2, p3);
        double angle2 = getAngle(p2, p3, p4);
        double angle3 = getAngle(p3, p4, p1);
        double angle4 = getAngle(p4, p1, p2);
        
        boolean isRightAngle = Math.abs(angle1 - 90) < 10 && Math.abs(angle2 - 90) < 10 &&
                               Math.abs(angle3 - 90) < 10 && Math.abs(angle4 - 90) < 10;
        
        return isValidSides && isRightAngle;
    }

    // Helper function to calculate the distance between two points
    private static double distance(Point p1, Point p2) {
        return Math.hypot(p2.x - p1.x, p2.y - p1.y);
    }

    // Helper function to calculate the angle between three points (p1-p2-p3)
    private static double getAngle(Point p1, Point p2, Point p3) {
        double angle = Math.toDegrees(Math.atan2(p3.y - p2.y, p3.x - p2.x) - Math.atan2(p1.y - p2.y, p1.x - p2.x));
        if (angle < 0) angle += 360;
        return angle;
    }

    // Function to detect rectangles from corner points
    public static List<RotatedRect> detectRectangles(List<Point> contourPoints) {
        List<RotatedRect> rectangles = new ArrayList<>();

        // Generate all combinations of 4 points from the contourPoints
        List<List<Point>> combinations = generateCombinations(contourPoints, 4);

        for (List<Point> points : combinations) {
            // Check if the combination of 4 points forms a rectangle
            if (isRectangle(points.get(0), points.get(1), points.get(2), points.get(3))) {
                // Create and store a RotatedRect for valid rectangle
                MatOfPoint2f matOfPoints = new MatOfPoint2f(points.toArray(new Point[0]));
                RotatedRect rect = Imgproc.minAreaRect(matOfPoints);
                rectangles.add(rect);
            }
        }
        return rectangles;
    }

    // Helper function to generate combinations of k points from the list
    public static List<List<Point>> generateCombinations(List<Point> points, int k) {
        List<List<Point>> combinations = new ArrayList<>();
        combinationHelper(points, new ArrayList<>(), k, 0, combinations);
        return combinations;
    }

    private static void combinationHelper(List<Point> points, List<Point> temp, int k, int start, List<List<Point>> result) {
        if (temp.size() == k) {
            result.add(new ArrayList<>(temp));
            return;
        }
        for (int i = start; i < points.size(); i++) {
            temp.add(points.get(i));
            combinationHelper(points, temp, k, i + 1, result);
            temp.remove(temp.size() - 1);
        }
    }

    public static void main(String[] args) {
        // Example corner points, in practice these would come from contour detection
        List<Point> contourPoints = Arrays.asList(
            new Point(10, 10), new Point(20, 10), new Point(30, 10), new Point(10, 20),
            new Point(20, 20), new Point(30, 20), new Point(10, 30), new Point(20, 30)
        );

        // Detect rectangles from the points
        List<RotatedRect> rectangles = detectRectangles(contourPoints);

        // Output the found rectangles
        for (RotatedRect rect : rectangles) {
            System.out.println("Detected rectangle: Center: " + rect.center + " Size: " + rect.size + " Angle: " + rect.angle);
        }
    }
}
