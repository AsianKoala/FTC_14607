package org.firstinspires.ftc.teamcode.util;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.jetbrains.annotations.NotNull;

public class Pose extends Point {
    public double heading;

    public Pose(double x, double y, double heading) {
        super(x, y);
        this.heading = heading;
    }

    public Pose(Point p, double heading) {
        this(p.x, p.y, heading);
    }

    public Pose add(Pose p2) {
        return new Pose(x + p2.x, y + p2.y, Util.angleWrap(heading + p2.heading));
    }

    public Pose(Pose2d pose2d) {
        this(pose2d.getX(), pose2d.getY(), pose2d.getHeading());
    }

    @NotNull
    @SuppressLint("DefaultLocale")
    @Override
    public String toString() {
        return String.format("{x: %.3f, y: %.3f, θ: %.3f}", x, y, Math.toDegrees(heading));
    }
}