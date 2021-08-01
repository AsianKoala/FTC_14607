package org.firstinspires.ftc.teamcode.control.controllers;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.control.path.LockedPathPoint;
import org.firstinspires.ftc.teamcode.control.path.PathPoint;
import org.firstinspires.ftc.teamcode.control.system.Azusa;
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.Point;
import org.firstinspires.ftc.teamcode.util.Pose;

public class PurePursuitController {
    private static final Pose mins = new Pose(0.11, 0.09, 0.11);

    public static void goToPosition(Azusa azusa, PathPoint target, PathPoint start) {
        Pose relVals = azusa.currPose.relVals(target);
        azusa.packet.addData("relX", relVals.x);
        azusa.packet.addData("relY", relVals.y);

        double smoothing = 12;
        double v = relVals.abs().x + relVals.abs().y;
        Pose powerPose = relVals.multiply(relVals).scale(1/(v * smoothing));

        double relAngle = getDesiredAngle(azusa.currPose, target);
        azusa.packet.addData("relH", Math.toDegrees(relAngle));

        powerPose.h = MathUtil.angleWrap(target.minus(start).atan() - azusa.currPose.h) / Math.toRadians(35);
        if(target instanceof LockedPathPoint) {
            powerPose.h = MathUtil.angleWrap(((LockedPathPoint) target).h - azusa.currPose.h) / Math.toRadians(35);
        }

        // checks and further smoothings
        boolean turning = true;
        if(relVals.hypot() < 3) {
            powerPose.h = 0;
            turning = false;
        }

        powerPose = powerPose.minify(mins);
        powerPose = powerPose.multiply(new Pose(
                Range.clip(relVals.x/3.0,0,1),
                Range.clip(relVals.y/3.0,0,1),
                Range.clip(Math.abs(powerPose.h)/Math.toRadians(5),0,1)));

        double turnErrorScaler = turning?Range.clip(1.0-Math.abs(relAngle/Math.toRadians(40)),0.4,0.8):1;
        azusa.packet.addData("turnErrorScalar", turnErrorScaler);
        powerPose.x *= turnErrorScaler;
        powerPose.y *= turnErrorScaler;

//        powerPose.x *= 1 - Range.clip(Math.abs(powerPose.h),0,0.8);
//        powerPose.y *= 1 - Range.clip(Math.abs(powerPose.h),0,0.8);

        azusa.driveTrain.powers = powerPose.clipAbs(1.0);
    }

    private static double getDesiredAngle(Pose curr, PathPoint target) {
        double forward = target.minus(curr).atan();
        double back = forward + Math.PI;
        double angleToForward = MathUtil.angleWrap(forward - curr.h);
        double angleToBack = MathUtil.angleWrap(back - curr.h);
        double autoAngle = Math.abs(angleToForward) < Math.abs(angleToBack) ? forward : back;
        return MathUtil.angleWrap(autoAngle - curr.h);
    }


    public static void followPath(Azusa azusa, PathPoint start, PathPoint end) {
        Point clip = MathUtil.clipIntersection2(start, end, azusa.currPose);
        Point intersectPoint = MathUtil.circleLineIntersection(clip, start, end, end.followDistance);

        PathPoint followPoint = new PathPoint(end);
        followPoint.x = intersectPoint.x;
        followPoint.y = intersectPoint.y;
        azusa.packet.fieldOverlay()
                .setFill("white")
                .fillCircle(followPoint.dbNormalize().x, followPoint.dbNormalize().y,2);
        goToPosition(azusa, followPoint, start);
    }
}
