package org.firstinspires.ftc.teamcode.control.path;

import java.util.LinkedList;

public class Path extends LinkedList<PathPoints.BasePathPoint> {
    public boolean isPurePursuit;

    public Path(Path path) {
        for (PathPoints.BasePathPoint pathPoint : path) add(new PathPoints.BasePathPoint(pathPoint));
        isPurePursuit = true;
    }

    public Path() {
        super();
        isPurePursuit = true;
    }

}