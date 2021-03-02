package org.firstinspires.ftc.teamcode.control.controllers;

import java.util.LinkedList;

public class Path {
    public LinkedList<PathPoint> pathPoints;

    public Path(PathBuilder builder) {
        for (PathPoint pathPoint : builder.pathPoints) pathPoints.add(pathPoint.clone());
    }

}