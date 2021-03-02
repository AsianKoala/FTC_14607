package org.firstinspires.ftc.teamcode.control.path.builders;

import org.firstinspires.ftc.teamcode.control.path.Path;
import org.firstinspires.ftc.teamcode.control.path.PathPoint;

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
