package org.firstinspires.ftc.teamcode.control.controllers;

public class PathPointBuilder {
    public PathPoint p;

    public PathPointBuilder(double x, double y, double followDistance, double heading) {
        p = new PathPoint(x,y,followDistance,heading);
    }

    public PathPointBuilder addFunc(Function function) {
        p.functions.add(function);
        return this;
    }

    public PathPoint build() {
        return p.clone();
    }
}