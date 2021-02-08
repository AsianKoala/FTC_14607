package org.firstinspires.ftc.teamcode.util;

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


    @SuppressLint("DefaultLocale")
    @NotNull
    @Override
    public String toString() {
        return String.format("(%.1f, %.1f)", x, y);
    }

}