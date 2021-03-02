package org.firstinspires.ftc.teamcode.control.path.builders;

import org.firstinspires.ftc.teamcode.control.path.Function;
import org.firstinspires.ftc.teamcode.control.path.PathPoint;

public class PathPointBuilder {
    public PathPoint p;

    public PathPointBuilder(PathPoint point) {
        p = point.clone();
    }

    public PathPointBuilder addFunc(Function function) {
        p.functions.add(function);
        return this;
    }

    public PathPoint build() {
        return p.clone();
    }
}