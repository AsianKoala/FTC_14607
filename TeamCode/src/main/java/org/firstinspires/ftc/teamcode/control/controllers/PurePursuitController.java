package org.firstinspires.ftc.teamcode.control.controllers;

import org.firstinspires.ftc.teamcode.control.system.Robot;
import org.firstinspires.ftc.teamcode.hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.Point;
import org.firstinspires.ftc.teamcode.util.Pose;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.control.path.PathPoints.*;
import static org.firstinspires.ftc.teamcode.util.MathUtil.*;

public class PurePursuitController {

    public static void goToPosition(Robot robot, BasePathPoint target, BasePathPoint finalTarget, BasePathPoint start) {
        Pose powerPose = new Pose();

        int index = 0;
        while(index < target.getTypeList().length-1 && target.getTypeList()[index] == null) {
            index++;
        }
        types pathPointType = types.values()[index];


        if(finalTarget == null) {
            Pose relVals = Robot.currPose.relVals(target);

            double v = relVals.abs().x + relVals.abs().y;
            powerPose.x = relVals.abs().x / 30;
            powerPose.y = relVals.abs().y / 30;
            powerPose.x *= relVals.x / v;
            powerPose.y *= relVals.y / v;

            powerPose.set(powerPose);

            if(target.lateTurnPoint == null) {
                powerPose.heading = getDesiredAngle(Robot.currPose, target, pathPointType.isLocked());
            } else {
                if(start.distance(Robot.currPose) > start.distance(target.lateTurnPoint)) {
                    powerPose.heading = getDesiredAngle(Robot.currPose, target, true);
                } else {
                    powerPose.heading = getDesiredAngle(Robot.currPose, target, false);
                }
            }

            System.out.println("AS FAST AS POSSIBLE");
        } else {

            Pose relVals = Robot.currPose.relVals(finalTarget);
            Pose relLineVals = new Pose(start, start.subtract(finalTarget).atan()).relVals(new Pose(finalTarget, 0));
            relLineVals.set(relLineVals.abs());

            double smoothinx = relLineVals.x * 0.8 > 30 ? relLineVals.x * 0.8 : 30;
            double smoothiny = relLineVals.y * 0.8 > 30 ? relLineVals.y * 0.8 : 30;

            double v = relVals.abs().x + relVals.abs().y;
            powerPose.x = relVals.abs().x / smoothinx;
            powerPose.y = relVals.abs().y / smoothiny;
            powerPose.x *= relVals.x / v;
            powerPose.y *= relVals.y / v;

            powerPose.set(powerPose);

            powerPose.heading = getDesiredAngle(Robot.currPose, finalTarget, true);

            if(finalTarget.lateTurnPoint == null) {
                powerPose.heading = getDesiredAngle(Robot.currPose, finalTarget, pathPointType.isLocked());
            } else {
                if(start.distance(Robot.currPose) > start.distance(finalTarget.lateTurnPoint)) {
                    powerPose.heading = getDesiredAngle(Robot.currPose, finalTarget, true);
                } else {
                    powerPose.heading = getDesiredAngle(Robot.currPose, finalTarget, false);
                }
            }

            System.out.println("SMOOTHING POWERPOSE: " + powerPose);
            System.out.println("SMOOTHING FINALTARGET RELVELS: " + relVals);
        }


        System.out.println("currpose: " + Robot.currPose);
        System.out.println("target: " + target.toString());
        System.out.println("finalTarget: " + (finalTarget == null ? "" : finalTarget));
        System.out.println("curr vel: " + robot.currVel);
        System.out.println("vel hypot: " + robot.currVel.hypot());
        DriveTrain.powers.set(powerPose);

    }

    private static double getDesiredAngle(Pose curr, BasePathPoint target, boolean locked) {
        double forward = target.subtract(curr).atan();
        double back = forward + Math.PI;
        double angleToForward = MathUtil.angleWrap(forward - curr.heading);
        double angleToBack = MathUtil.angleWrap(back - curr.heading);
        double autoAngle = Math.abs(angleToForward) < Math.abs(angleToBack) ? forward : back;
        double desired =  locked ? target.lockedHeading : autoAngle;
        return angleWrap(desired - curr.heading) / Math.toRadians(40);
    }

    public static boolean runFuncList(BasePathPoint target) {
        target.functions.removeIf(f -> f.cond() && f.func());
        return target.functions.size() == 0;
    }

    public static void followPath(Robot robot, BasePathPoint start, BasePathPoint end, ArrayList<BasePathPoint> allPoints) {
        Point clip = MathUtil.clipIntersection2(start, end, Robot.currPose);
        Point intersectPoint = MathUtil.circleLineIntersection(clip, start, end, end.followDistance);

        BasePathPoint followPoint = new BasePathPoint(end);
        followPoint.x = intersectPoint.x;
        followPoint.y = intersectPoint.y;

        goToPosition(robot, followPoint, end.isStop != null ? end : null, start);
    }
}
