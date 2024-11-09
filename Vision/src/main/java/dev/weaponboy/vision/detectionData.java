package dev.weaponboy.vision;

import org.opencv.core.Point;

public class detectionData {

    double readTime;
    Point targetPoint;
    double angle;

    public detectionData(double readTime, Point targetPoint, double angle){
        this.readTime = readTime;
        this.targetPoint = targetPoint;
        this.angle = angle;
    }

    public double getReadTime() {
        return readTime;
    }

    public Point getTargetPoint() {
        return targetPoint;
    }

    public double getAngle() {
        return angle;
    }

}
