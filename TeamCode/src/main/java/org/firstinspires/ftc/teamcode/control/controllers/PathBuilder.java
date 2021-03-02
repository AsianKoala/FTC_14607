package org.firstinspires.ftc.teamcode.control.controllers;

public class PathBuilder {
    public Path path;

    public PathBuilder() {
        path = new Path();
    }

    public PathBuilder addPoint(PathPoint p) {
        path.add(p);
        return this;
    }

    public Path build() {
        return new Path(path);
    }
}
