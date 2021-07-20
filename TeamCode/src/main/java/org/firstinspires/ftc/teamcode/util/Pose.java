package org.firstinspires.ftc.teamcode.util;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.geometry.Pose2d;

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

    public Pose(Pose2d pose2d) {
        this(pose2d.getX(), pose2d.getY(), pose2d.getHeading());
    }

    public Pose() {
        this(0, 0, 0);
    }

    public Pose(double a) { this(a,a,a); }

    public Pose(Point p, double heading) { this(p.x, p.y, heading); }

    // im going to trust myself to angle wrap if it comes to it
    public Pose add(Pose p) {
        return new Pose(super.add(p), heading+p.heading);
    }

    public Pose minus(Pose p) {
        return this.add(new Pose(-p.x, -p.y, -p.heading));
    }

    public Pose multiply(Pose p) {
        return divide(new Pose(1/p.x, 1/p.y, 1/p.heading));
    }

    public Pose divide(Pose p) {
        return new Pose(super.divide(p), heading/p.heading);
    }

    public Pose abs() {
        return new Pose(super.abs(), Math.abs(heading));
    }

    public Pose sgns() {
        return new Pose(sgn(x), sgn(y), sgn(heading));
    }

    public Pose pow(Pose p) {
        return new Pose(super.pow(this), Math.pow(heading, p.heading));
    }

    public double cos() {
        return Math.cos(heading);
    }

    public double sin() {
        return Math.sin(heading);
    }

    public Pose relVals(Point target) {
        double d = minus(target).hypot();
        double r_h = target.minus(this).atan() - heading;
        double r_x = -d * Math.sin(r_h);
        double r_y = d * Math.cos(r_h);
        return new Pose(r_x, r_y, r_h); // return 0 just to kmake sure neer to use it kek
    }

    public Pose wrap() {
        return new Pose(x, y, MathUtil.angleWrap(heading));
    }

    public void set(Pose p) {
        x = p.x;
        y = p.y;
        heading = p.heading;
    }


    @SuppressLint("DefaultLocale")
    @Override
    public String toString() {
        return String.format("(%.1f, %.1f, %.1f)", x, y, heading);
    }

}
