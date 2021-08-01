package org.firstinspires.ftc.teamcode.control.path;

import org.firstinspires.ftc.teamcode.control.system.Functions;

public class OnlyTurnPoint extends LockedPathPoint {
    public double dh;
    public OnlyTurnPoint(String signature, double x, double y, double followDistance, double h, double dh) {
        super(signature, x, y, followDistance, h);
        this.dh = dh;
    }

    public OnlyTurnPoint(String signature, double x, double y, double followDistance, double h, double dh, Functions.Function func) {
        super(signature, x, y, followDistance, h, func);
        this.dh = dh;
    }

    public OnlyTurnPoint(OnlyTurnPoint p) {
        this(p.signature, p.x, p.y, p.followDistance, p.h, p.dh, p.func);
    }
}
