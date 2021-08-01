package org.firstinspires.ftc.teamcode.control.path;

import android.annotation.SuppressLint;

import org.firstinspires.ftc.teamcode.control.system.Functions;
import org.firstinspires.ftc.teamcode.util.Point;

public class PathPoint extends Point {
    public double followDistance;
    public String signature;
    public Functions.Function func;

    public PathPoint(String signature, double x, double y, double followDistance) {
        this(signature, x, y, followDistance, null);
    }
    public PathPoint(String signature, double x, double y, double followDistance, Functions.Function func) {
        super(x, y);
        this.signature = signature;
        this.followDistance = followDistance;
        this.func = func;
    }

    public PathPoint(PathPoint b) {
        this(b.signature, b.x, b.y, b.followDistance, b.func);
    }

    @SuppressLint("DefaultLocale")
    @Override
    public String toString() {
        return String.format("%s, %.1f, %.1f, %.1f", signature, x, y, followDistance);
    }

    public boolean equals(PathPoint b) {
        return x == b.x && y == b.y;
    }
}//sdd