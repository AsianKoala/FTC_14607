package org.firstinspires.ftc.teamcode.util;

import android.annotation.SuppressLint;

public class Point {
    public double x, y;

    public Point(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Point(Point p) {
        this(p.x, p.y);
    }

    public Point(double a) { this(a,a); }

    public Point() {
        this(0, 0);
    }

    public Point rotated(double angle) {
        double newX = x * Math.cos(angle) - y * Math.sin(angle);
        double newY = x * Math.sin(angle) + y * Math.cos(angle);
        return new Point(newX, newY);
    }

    public Point add(Point p) {
        x += p.x;
        y += p.y;
        return new Point(this);
    }

    public Point subtract(Point p) {
        return add(new Point(-p.x, -p.y));
    }

    public Point divide(Point p) {
        if(x == 0 || y == 0)
            throw new IllegalArgumentException("zero division");
        x /= p.x;
        y /= p.y;
        return new Point(this);
    }

    public Point multiply(Point p) {
        return divide(new Point(1/p.x, 1/p.y));
    }

    public Point pow(Point p) {
        x = Math.pow(x, p.x);
        y = Math.pow(y, p.y);
        return new Point(this);
    }

    public Point abs() {
        return new Point(Math.abs(x), Math.abs(y));
    }

    public Point sgns() {
        return new Point(MathUtil.sgn(x), MathUtil.sgn(y));
    }

    public double hypot() {
        return Math.sqrt(x * x + y * y);
    }

    public double distance(Point p) {
        return Math.hypot(x - p.x, y - p.y);
    }

    public double atan() { return Math.atan2(y, x); }

    @SuppressLint("DefaultLocale")
    @Override
    public String toString() {
        return String.format("(%.1f, %.1f)", x, y);
    }

}
