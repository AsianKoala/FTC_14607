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

    public Point add(double a) {
        x += a;
        y += a;
        return new Point(this);
    }

    public double hypot() {
        return Math.sqrt(x * x + y * y);
    }

    public double distance(Point p) {
        return Math.hypot(x - p.x, y - p.y);
    }

    @Override
    @SuppressLint("DefaultLocale")
    public String toString() {
        return String.format("(%.1f, %.1f)", x, y);
    }

}
