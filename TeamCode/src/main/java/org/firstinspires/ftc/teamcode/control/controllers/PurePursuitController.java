package org.firstinspires.ftc.teamcode.control.controllers;

import org.firstinspires.ftc.teamcode.control.system.Robot;
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
            Pose relVals = robot.currPose.relVals(target);

            double v = relVals.abs().x + relVals.abs().y;
            powerPose.x = relVals.abs().x / 30;
            powerPose.y = relVals.abs().y / 30;
            powerPose.x *= relVals.x / v;
            powerPose.y *= relVals.y / v;

            powerPose.set(powerPose);

            if(target.lateTurnPoint == null) {
                powerPose.heading = getDesiredAngle(robot.currPose, target, pathPointType.isLocked());
            } else {
                if(start.distance(robot.currPose) > start.distance(target.lateTurnPoint)) {
                    powerPose.heading = getDesiredAngle(robot.currPose, target, true);
                } else {
                    powerPose.heading = getDesiredAngle(robot.currPose, target, false);
                }
            }

            System.out.println("AS FAST AS POSSIBLE");
        } else {

            Pose relVals = robot.currPose.relVals(finalTarget);
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

            powerPose.heading = getDesiredAngle(robot.currPose, finalTarget, true);

            if(finalTarget.lateTurnPoint == null) {
                powerPose.heading = getDesiredAngle(robot.currPose, finalTarget, pathPointType.isLocked());
            } else {
                if(start.distance(robot.currPose) > start.distance(finalTarget.lateTurnPoint)) {
                    powerPose.heading = getDesiredAngle(robot.currPose, finalTarget, true);
                } else {
                    powerPose.heading = getDesiredAngle(robot.currPose, finalTarget, false);
                }
            }

            System.out.println("SMOOTHING POWERPOSE: " + powerPose);
            System.out.println("SMOOTHING FINALTARGET RELVELS: " + relVals);
        }


        System.out.println("currpose: " + robot.currPose);
        System.out.println("target: " + target.toString());
        System.out.println("finalTarget: " + (finalTarget == null ? "" : finalTarget));
        System.out.println("curr vel: " + robot.currVel);
        System.out.println("vel hypot: " + robot.currVel.hypot());
        robot.driveTrain.powers.set(powerPose);

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

    static double powRetainingSign(double a, double b) {
        return sgn(a) * Math.pow(Math.abs(a), b);
    }

    public static boolean runFuncList(BasePathPoint target) {
        target.functions.removeIf(f -> f.cond() && f.func());
        return target.functions.size() == 0;
    }

    public static void followPath(Robot robot, BasePathPoint start, BasePathPoint end, ArrayList<BasePathPoint> allPoints) {
        Point clip = MathUtil.clipIntersection2(start, end, robot.currPose);
        Point intersectPoint = MathUtil.circleLineIntersection(clip, start, end, end.followDistance);

        BasePathPoint followPoint = new BasePathPoint(end);
        followPoint.x = intersectPoint.x;
        followPoint.y = intersectPoint.y;

        goToPosition(robot, followPoint, end.isStop != null ? end : null, start);
    }
}
//
//    public static boolean goToPosition(Robot robot, BasePathPoint target) {
//        double d = robot.currPose.distance(target);
//        Pose relVals = robot.currPose.relDistance(target);
//        boolean done;
//
//        int index = 0;
//        System.out.println(Arrays.toString(target.getTypeList()));
//        while(index < target.getTypeList().length-1 && target.getTypeList()[index] == null) {
//            index++;
//        }
//        types pathPointType = types.values()[index];
//
//        Pose powerPose = new Pose();
//        double v = relVals.abs().x + relVals.abs().y;
//        Pose move = new Pose();
//        move.x = relVals.abs().x / 60;
//        move.y = relVals.abs().y / 60;
//        move.x *= relVals.x / v;
//        move.y *= relVals.y / v;
//
//        powerPose.set(move);
//
//        double targetAngle = pathPointType.isLocked() ? target.lockedHeading : target.subtract(robot.currPose).atan();
//        double angleToTarget = angleWrap(targetAngle - robot.currPose.heading);
//        powerPose.heading = angleToTarget / Math.toRadians(45);
//
//        System.out.println("index: " + index);
//        System.out.println("type: " + pathPointType.toString());
//
//        if(pathPointType.ordinal() == types.lateTurn.ordinal() &&
//                target.distance(target.lateTurnPoint) < target.distance(robot.currPose)) {
//            powerPose.heading = 0;
//            done = d < 2 && MathUtil.angleThresh(robot.currPose.heading, target.lockedHeading);
//        } else if(pathPointType.ordinal() == types.onlyTurn.ordinal()) {
//            powerPose.x = 0;
//            powerPose.y = 0;
//            done = MathUtil.angleThresh(robot.currPose.heading, target.lockedHeading);
//        } else if(pathPointType.ordinal() == types.onlyFunctions.ordinal()) {
//            powerPose = new Pose(0,0,0);
//            done = target.functions.size() == 0;
//        } else {
//            done = d < 2 && MathUtil.angleThresh(robot.currPose.heading, target.lockedHeading);
//        }
//
//        target.functions.removeIf(f -> f.cond() && f.func());
//
//        done = done && target.functions.size() == 0;
//
//
//        robot.currDrivePowers = powerPose;
//
//        System.out.println("relVel: " + robot.currVel.toString());
//        System.out.println("VEL: " + robot.currVel.hypot());
//        System.out.println("powePose: " + powerPose);
//        System.out.print("D: " + d);
//        System.out.println();
//        System.out.println();
//
//        return done;
//    }
//
//    public static void followPath(Robot robot) {
//
//
//    }
//}
//
//
//
//
//
//        if(!target.stop || d > 45) {
//        powerPose.set(powerPose.divide(new Pose(v)).multiply(relVals.abs().divide(new Pose(41)))); // someting wrong here prob
//        relVals.set(relVals.divide(new Pose(v)));

//        if (OpModeClock.isOk()) {
//                System.out.println("speed");
//                System.out.println("currVel: " + robot.currVel.toString());
//                System.out.println("currVelHypot: " + robot.currVel.hypot());
////                System.out.println("relVals: " + relVals.toString());
////                System.out.println("angle to target: " + Math.toDegrees(angleToTarget));
////                System.out.println("powerPose: " + powerPose);
//                System.out.println();
//                System.out.println();
//            }
//        } else if(robot.currVel.hypot() > 20 && d > smoothDist) {
////            double x1 = target.x + (d / smoothDist) * (robot.currPose.x - target.x);
////            double y1 = target.y + (d / smoothDist) * (robot.currPose.y - target.y);
////            target.x = x1;
////            target.y = y1;
////
////            relVals = robot.currPose.relDistance(target);
////            double v = relVals.abs().x + relVals.abs().y;
////            Pose powerPose = new Pose(relVals.x, relVals.y, 69420); // heading doesnt matter for now so ya
////            powerPose.set(powerPose.divide(new Pose(v)).multiply(relVals.abs().divide(new Pose(45))));
////
////            double desiredAngle = !target.locked ? target.subtract(robot.currPose).atan() : target.heading;
////            double angleToTarget = angleWrap(desiredAngle - robot.currPose.heading);
////            powerPose.heading = angleToTarget / Math.toRadians(45);
////
////            powerPose.x *= Range.clip(1.0-(angleToTarget/Math.toRadians(45)), 0.5, 1);
////            powerPose.y *= Range.clip(1.0-(angleToTarget/Math.toRadians(45)), 0.5, 1);
////
////            robot.speeds = powerPose;
////
////            if (Omodeclock.isOk()) {
////                System.out.println("adjusted  speed");
////                System.out.println("relVals: " + relVals.toString());
////                System.out.println("angle to target: " + Math.toDegrees(angleToTarget));
////                System.out.println("powerPose: " + powerPose);
////                System.out.println();
////                System.out.println();
////            }
//        } else { // rel vals in total will be 6, clip to max and divide and maintain shape
//            relVals = robot.currPose.relDistance(target);
//
//            double startPower = 1;
//            double endPower = 0.15;
//            double slope = (startPower - endPower) / (smoothDist - 1);
//            double intercept = startPower - smoothDist * slope;
//
//            powerPose.set(powerPose.multiply(new Pose(slope)).add(new Pose(intercept)));
//
//            powerPose.heading = angleToTarget / Math.toRadians(60);
//            powerPose.heading = Range.clip(powerPose.heading, -0.3, 0.3);
//
//            robot.speeds = powerPose;
//
//            if (OpModeClock.isOk()) {
//                System.out.println("super slow");
//                System.out.println("relVals: " + relVals.toString());
//                System.out.println("angle to target: " + Math.toDegrees(angleToTarget));
//                System.out.println("powerPose: " + powerPose);
//                System.out.println();
//                System.out.println();
//            }
//        }
