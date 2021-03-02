package org.firstinspires.ftc.teamcode.control.path;

import org.firstinspires.ftc.teamcode.util.Pose;
import org.jetbrains.annotations.NotNull;

import java.util.LinkedList;

public class PathPoint extends Pose implements Cloneable {
    public double followDistance;
    public LinkedList<Function> functions;
    public PathPointMod mod;

    public PathPoint(double x, double y, double heading, double followDistance, PathPointMod mod) {
        super(x, y, heading);
        this.followDistance = followDistance;
        functions = new LinkedList<>();
        this.mod = mod;
    }

    public PathPoint(Pose pose, double followDistance) {
        super(pose);
        this.followDistance = followDistance;
        functions = new LinkedList<>();
    }

    @NotNull
    @Override
    public PathPoint clone() {
        PathPoint p = new PathPoint(x, y, heading, followDistance, mod);
        p.functions = functions;
        return p;
    }
}

