package org.firstinspires.ftc.teamcode.control.path.builders;

import org.firstinspires.ftc.teamcode.control.path.Path;
import org.firstinspires.ftc.teamcode.control.path.PathPoints.BasePathPoint;

public class PathBuilder {
    public Path path;

    public PathBuilder(String name) {
        path = new Path(name);
    }

    public PathBuilder addPoint(BasePathPoint p) {
        path.add(p);
        return this;
    }

    public Path build() {
        if(path.size()==1)
            return new Path(path.getFirst(), path.name);
        return new Path(path);
    }
}
