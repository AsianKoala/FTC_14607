package org.firstinspires.ftc.teamcode.control.path;

import java.util.LinkedList;

public class Path extends LinkedList<PathPoint> {
    public boolean isPurePursuit;

    public Path(Path path) {
        for (PathPoint pathPoint : path) add(pathPoint.clone());
        isPurePursuit = true;
    }

    public Path() {
        super();
        isPurePursuit = true;
    }

}