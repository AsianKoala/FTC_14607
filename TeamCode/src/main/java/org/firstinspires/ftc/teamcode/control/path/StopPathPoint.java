package org.firstinspires.ftc.teamcode.control.path;

import org.firstinspires.ftc.teamcode.control.system.Functions;

public class StopPathPoint extends LockedPathPoint {
    public StopPathPoint(String signature, double x, double y, double followDistance, double h) {
        super(signature, x, y, followDistance, h);
    }

    public StopPathPoint(String signature, double x, double y, double followDistance, double h, Functions.Function func) {
        super(signature, x, y, followDistance, h, func);
    }

    public StopPathPoint(StopPathPoint p) {
        this(p.signature, p.x, p.y, p.followDistance, p.h, p.func);
    }
}