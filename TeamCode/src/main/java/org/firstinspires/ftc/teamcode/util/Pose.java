package org.firstinspires.ftc.teamcode.util;

import android.annotation.SuppressLint;

import static org.firstinspires.ftc.teamcode.util.MathUtil.*;

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

    public Pose(double a) { this(a,a,a); }

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

    public Pose multiply(Pose p) {
        x *= p.x;
        y *= p.y;
        heading *= p.heading;
        return new Pose(this);
    }

    public Pose divide(Pose p) {
        return multiply(new Pose(1/p.x, 1/p.y, 1/p.heading));
    }

    public Pose wrap() {
        heading = MathUtil.angleWrap(heading);
        return new Pose(this);
    }

    public double cos() {
        return Math.cos(heading);
    }

    public double sin() {
        return Math.sin(heading);
    }

    // make sure to wrap
    public Pose distancePose(Pose target) {
        double distance = subtract(target).hypot();
        Pose dH = new Pose(0, 0, subtract(target).atan() - heading);
        double dX = distance * dH.cos();
        double dY = distance * dH.sin();
        return new Pose(dX, dY, dH.heading);
    }

    public Pose clipMax(Pose max) {
        if (Math.abs(x) > max.x) {
            x = max.x;
        }
        if (Math.abs(y) > max.y) {
            y = max.y;
        }
        if (Math.abs(heading) > max.heading) {
            heading = max.heading;
        }


        return new Pose(multiply(new Pose(sgn(max.x), sgn(max.y), sgn(max.heading))));
    }

    public Pose abs() {
        return new Pose(Math.abs(x), Math.abs(y), Math.abs(heading));
    }

    public Pose set(Pose p) {
        x = p.x;
        y = p.y;
        heading = p.heading;
        return new Pose(this);
    }

    public Pose sgns() {
        return new Pose(sgn(x), sgn(y), sgn(heading));
    }

    public Pose pow(Pose p) {
        x = Math.pow(x, p.x);
        y = Math.pow(y, p.y);
        heading = Math.pow(heading, p.heading);
        return new Pose(this);
    }

    @SuppressLint("DefaultLocale")
    @Override
    public String toString() {
        return String.format("(%.1f, %.1f, %.1f)", x, y, Math.toRadians(heading));
    }

}
