package org.firstinspires.ftc.teamcode.control.controllers;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.util.Pose;
import org.jetbrains.annotations.NotNull;

import java.util.LinkedList;

public class PathPoint extends Pose implements Cloneable {
    public double followDistance;
    public LinkedList<Function> functions;

    public PathPoint(double x, double y, double heading, double followDistance) {
        super(x, y, heading);
        this.followDistance = followDistance;
        functions = new LinkedList<>();
    }

    public PathPoint(Pose pose, double followDistance) {
        super(pose);
        this.followDistance = followDistance;
    }

    public PathPoint() {
        this(0,0,-1000,0); // func only
    }

    @NotNull
    @Override
    public PathPoint clone() {
        PathPoint p = new PathPoint(x, y, heading, followDistance);
        p.functions = functions;
        return p;
    }

}


