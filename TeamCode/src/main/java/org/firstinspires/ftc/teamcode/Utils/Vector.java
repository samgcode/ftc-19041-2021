package org.firstinspires.ftc.teamcode.Utils;

public class Vector {
    public double x, y, z, h;

    public Vector(double x_, double y_) {
        x = x_;
        y = y_;
    }

    public Vector(double x_, double y_, double z_) {
        x = x_;
        y = y_;
        h = z_;
        z = z_;
    }

    public static Vector normalizeVector(Vector vector) {
        double angle = Math.toRadians(vector.h);
        double yComponent = (-(vector.x * Math.sin(angle)) + (vector.y * Math.cos(angle)));
        double xComponent = ((vector.x * Math.cos(angle)) + (vector.y * Math.sin(angle)));

        return new Vector(xComponent, yComponent);
    }
}