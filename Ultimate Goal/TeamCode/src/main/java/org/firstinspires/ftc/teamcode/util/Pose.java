package org.firstinspires.ftc.teamcode.util;

import android.annotation.SuppressLint;

import org.jetbrains.annotations.NotNull;

public class Pose extends Point implements Cloneable {
    public double heading;

    public Pose(double x, double y, double heading) {
        super(x, y);
        this.heading = heading;
    }

    public Pose(Point p, double heading) {
        this(p.x, p.y, heading);
    }



    @NotNull
    @SuppressLint("DefaultLocale")
    @Override
    public String toString() {
        return String.format("{x: %.3f, y: %.3f, θ: %.3f}", x, y, Math.toDegrees(heading));
    }
}