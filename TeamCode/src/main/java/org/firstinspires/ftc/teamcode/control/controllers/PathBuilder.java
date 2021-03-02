package org.firstinspires.ftc.teamcode.control.controllers;

import java.util.LinkedList;

public class PathBuilder {
    public LinkedList<PathPoint> pathPoints;

    public PathBuilder() {
        pathPoints = new LinkedList<>();
    }

    public PathBuilder addPoint(PathPoint p) {
        pathPoints.add(p);
        return this;
    }

    public Path build() {
        return new Path(this);
    }
}
