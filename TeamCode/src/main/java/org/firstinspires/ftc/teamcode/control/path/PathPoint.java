package org.firstinspires.ftc.teamcode.control.path;

import org.firstinspires.ftc.teamcode.util.Pose;
import org.jetbrains.annotations.NotNull;

import java.util.LinkedList;

public class PathPoint extends Pose implements Cloneable {
    public double followDistance;
    public LinkedList<Function> functions;
    public boolean locked;
    public boolean stop;

    public PathPoint(double x, double y, double heading, double followDistance, boolean locked, boolean stop) {
        super(x, y, heading);
        this.followDistance = followDistance;
        functions = new LinkedList<>();
        this.locked = locked;
        this.stop = stop;
    }

    public PathPoint(Pose pose, double followDistance, boolean locked, boolean stop) {
        this(pose.x, pose.y, pose.heading, followDistance, locked, stop);
    }

    @NotNull
    @Override
    public PathPoint clone() {
        PathPoint p = new PathPoint(x, y, heading, followDistance, locked, stop);
        p.functions.addAll(functions);
        return p;
    }
}

