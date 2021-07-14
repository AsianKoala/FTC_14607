package org.firstinspires.ftc.teamcode.control.path.builders;

import org.firstinspires.ftc.teamcode.control.path.Function;

import static org.firstinspires.ftc.teamcode.control.path.PathPoints.*;

public class PathPointBuilder {
    public BasePathPoint p;

    public PathPointBuilder(BasePathPoint point) {
        p = new BasePathPoint(point);
    }

    public PathPointBuilder addFunc(Function function) {
        p.functions.add(function);
        return this;
    }

    public BasePathPoint build() {
        return new BasePathPoint(p);
    }
}