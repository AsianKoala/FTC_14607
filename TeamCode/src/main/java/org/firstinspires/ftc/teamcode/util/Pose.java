package org.firstinspires.ftc.teamcode.util;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import static org.firstinspires.ftc.teamcode.util.MathUtil.*;

public class Pose extends Point {
    public double h;

    public Pose(double x, double y, double h) {
        super(x, y);
        this.h = h;
    }

    public Pose(Pose p) {
        this(p.x, p.y, p.h);
    }

    public Pose(Pose2d pose2d) {
        this(pose2d.getX(), pose2d.getY(), pose2d.getHeading());
    }

    public Pose() {
        this(0, 0, 0);
    }

    public Pose(double a) { this(a,a,a); }

    public Pose(Point p, double h) { this(p.x, p.y, h); }

    // im going to trust myself to angle wrap if it comes to it
    public Pose add(Pose p) {
        return new Pose(super.add(p), h +p.h);
    }

    public Pose minus(Pose p) {
        return this.add(new Pose(-p.x, -p.y, -p.h));
    }

    public Pose multiply(Pose p) {
        return divide(new Pose(1/p.x, 1/p.y, 1/p.h));
    }

    public Pose divide(Pose p) {
        return new Pose(super.divide(p), h /p.h);
    }

    public Pose abs() {
        return new Pose(super.abs(), Math.abs(h));
    }

    public Pose sgns() {
        return new Pose(sgn(x), sgn(y), sgn(h));
    }

    public Pose pow(Pose p) {
        return new Pose(super.pow(this), Math.pow(h, p.h));
    }

    public double cos() {
        return Math.cos(h);
    }

    public double sin() {
        return Math.sin(h);
    }

    public Pose relVals(Point target) {
        double d = minus(target).hypot();
        double r_h = target.minus(this).atan() - h;
        double r_x = -d * Math.sin(r_h);
        double r_y = d * Math.cos(r_h);
        return new Pose(r_x, r_y, r_h); // return 0 just to kmake sure neer to use it kek
    }

    public Pose wrap() {
        return new Pose(x, y, MathUtil.angleWrap(h));
    }

    public void set(Pose p) {
        x = p.x;
        y = p.y;
        h = p.h;
    }


    @SuppressLint("DefaultLocale")
    @Override
    public String toString() {
        return String.format("(%.1f, %.1f, %.1f)", x, y, h);
    }

}
