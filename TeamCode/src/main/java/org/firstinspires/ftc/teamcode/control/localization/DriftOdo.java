package org.firstinspires.ftc.teamcode.control.localization;

import org.firstinspires.ftc.teamcode.control.system.Azusa;
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.Point;
import org.firstinspires.ftc.teamcode.util.Pose;
import org.openftc.revextensions2.RevBulkData;

public class DriftOdo {

    private static final double TICKS_PER_INCH = 1103.8839;
    private final int perpPort = 0;
    private final int parallelPort = 2;
    private final Point startTicks;

    private final Pose currPose;
    private Point lastWheels;

    public DriftOdo(Pose startPose, RevBulkData data) {
        currPose = startPose;
        startTicks = new Point(data.getMotorCurrentPosition(perpPort),
                data.getMotorCurrentPosition(parallelPort));
        lastWheels = new Point(startTicks);
    }

    public Pose update(Azusa azusa, double imuH, RevBulkData data) {
        Point currW = new Point(data.getMotorCurrentPosition(perpPort),
                data.getMotorCurrentPosition(parallelPort));
        Point d = currW.minus(lastWheels).minus(startTicks);

        Point dScaled = d.scale(1 / TICKS_PER_INCH);
        double dH = MathUtil.angleWrap(imuH - currPose.h);
        currPose.h = imuH;

        double parallelCoeff = 1;
        double perpCoeff = 1;
        Point dAccounted = dScaled.minus(new Point(dH * perpCoeff, dH * parallelCoeff));
        currPose.x += (Math.cos(imuH) * dAccounted.y) + (Math.sin(imuH) * dAccounted.x);
        currPose.y += (Math.sin(imuH) * dAccounted.y) - (Math.cos(imuH) * dAccounted.x);

        lastWheels = currW;
        return currPose;
    }
}
