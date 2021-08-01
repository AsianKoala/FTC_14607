package org.firstinspires.ftc.teamcode.control.path;

import org.firstinspires.ftc.teamcode.control.system.Functions;

public class LockedPathPoint extends PathPoint {
    public double h;
    public LockedPathPoint(String signature, double x, double y, double followDistance, double h) {
        super(signature, x, y, followDistance);
        this.h = h;
    }

    public LockedPathPoint(String signature, double x, double y,  double followDistance, double h, Functions.Function func) {
        super(signature, x, y, followDistance, func);
        this.h = h;
    }

    public LockedPathPoint(LockedPathPoint p) {
        this(p.signature, p.x, p.y, p.followDistance, p.h, p.func);
    }
}