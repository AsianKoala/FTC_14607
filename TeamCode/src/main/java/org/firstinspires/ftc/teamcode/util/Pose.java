package org.firstinspires.ftc.teamcode.util;

import android.annotation.SuppressLint;

public class Pose extends Point {
    public double heading;

    public Pose(double x, double y, double heading) {
        super(x, y);
        this.heading = heading;
    }

    public Pose(Pose p) {
        this(p.x, p.y, p.heading);
    }

    public Pose() {
        this(0, 0, 0);
    }

    public Pose(Point p, double heading) {
        this(p.x, p.y, heading);
    }

    // im going to trust myself to angle wrap if it comes to it
    public Pose add(Pose p) {
        x += p.x;
        y += p.y;
        heading += p.heading;
        return new Pose(this);
    }

    public Pose subtract(Pose p) {
        return add(new Pose(-p.x, -p.y, -p.heading));
    }

    public Pose add(double a) {
        return add(new Pose(a, a, a));
    }

    public Pose subtract(double a) {
        return subtract(new Pose(a, a, a));
    }

    public Pose divide(double a) {
        if (a == 0) {
            throw new IllegalArgumentException("Attempted to divide by zero through pose division");
        }
        x /= a;
        y /= a;
        heading /= a;
        return new Pose(this);
    }

    public void wrap() {
        heading = MathUtil.angleWrap(heading);
    }

    @SuppressLint("DefaultLocale")
    @Override
    public String toString() {
        return String.format("(%.1f, %.1f, %.1f)", x, y, Math.toRadians(heading));
    }

}
