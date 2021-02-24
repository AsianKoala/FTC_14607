package org.firstinspires.ftc.teamcode.main.util;

import android.annotation.SuppressLint;

import org.jetbrains.annotations.NotNull;

public class Point {
    public double x;
    public double y;

    public Point() {
        x = 0;
        y = 0;
    }

    public Point(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Point rotated(double angle) {
        double newX = x * Math.cos(angle) - y * Math.sin(angle);
        double newY = x * Math.sin(angle) + y * Math.cos(angle);
        return new Point(newX, newY);
    }

    @SuppressLint("DefaultLocale")
    @NotNull
    @Override
    public String toString() {
        return String.format("(%.1f, %.1f)", x, y);
    }

}