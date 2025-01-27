package dev.weaponboy.command_library.Hardware;

import org.opencv.core.Point;

import dev.weaponboy.nexus_pathing.RobotUtilities.Vector2D;

public class TargetSample {

    Vector2D targetPoint;
    double angle;

    public TargetSample(Vector2D targetPoint, double angle){
        this.targetPoint = targetPoint;
        this.angle = angle;
    }

    public Vector2D getTargetPoint() {
        return targetPoint;
    }

    public double getAngle() {
        return angle;
    }
}
