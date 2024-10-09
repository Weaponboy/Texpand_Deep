package dev.weaponboy.command_library.Hardware;

public class collectionTarget {

    double X;
    double Y;
    double Z;

    public collectionTarget(){
        this.X = 0;
        this.Y = 0;
        this.Z = 0;
    }

    public collectionTarget(double X, double Y, double Z){
        this.X = X;
        this.Y = Y;
        this.Z = Z;
    }

    public double getX() {
        return X;
    }

    public void setX(double x) {
        X = x;
    }

    public double getY() {
        return Y;
    }

    public void setY(double y) {
        Y = y;
    }

    public double getZ() {
        return Z;
    }

    public void setZ(double z) {
        Z = z;
    }

}
