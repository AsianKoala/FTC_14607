package org.firstinspires.ftc.teamcode.control.controllers;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.control.system.Robot;
import org.firstinspires.ftc.teamcode.hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.Point;
import org.firstinspires.ftc.teamcode.util.Pose;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.control.path.PathPoints.*;
import static org.firstinspires.ftc.teamcode.control.system.Robot.currPose;
import static org.firstinspires.ftc.teamcode.hardware.DriveTrain.powers;
import static org.firstinspires.ftc.teamcode.util.MathUtil.*;

public class PurePursuitController {
    public static double movement_y_min = 0.091;
    public static double movement_x_min = 0.11;
    public static double movement_turn_min = 0.10;

    private static void allComponentsMinPower() {
        if(Math.abs(powers.x) > Math.abs(powers.y)){
            if(Math.abs(powers.x) > Math.abs(powers.heading)){
                powers.x = minPower(powers.x,movement_x_min);
            }else{
                powers.heading = minPower(powers.heading,movement_turn_min);
            }
        }else{
            if(Math.abs(powers.y) > Math.abs(powers.heading)){
                powers.y = minPower(powers.y, movement_y_min);
            }else{
                powers.heading = minPower(powers.heading,movement_turn_min);
            }
        }
    }

    public static double minPower(double val, double min){
        if(val >= 0 && val <= min){
            return min;
        }
        if(val < 0 && val > -min){
            return -min;
        }
        return val;
    }



    private static double getDesiredAngle(Pose curr, BasePathPoint target, boolean locked) {
        double forward = target.minus(curr).atan();
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

    public static void goToPosition(Robot robot, BasePathPoint target, BasePathPoint finalTarget, BasePathPoint start) {
        Pose powerPose = new Pose();

        int index = 0;
        while(index < target.getTypeList().length-1 && target.getTypeList()[index] == null) {
            index++;
        }
        types pathPointType = types.values()[index];


        if(finalTarget == null) {
            Pose relVals = Robot.currPose.relVals(target);
            robot.packet.put("relX", relVals.x);
            robot.packet.put("relY", relVals.y);

            double v = relVals.abs().x + relVals.abs().y;
            powerPose.x = relVals.abs().x / 30;
            powerPose.y = relVals.abs().y / 30;
            powerPose.x *= relVals.x / v;
            powerPose.y *= relVals.y / v;

            powerPose.set(powerPose);

            if(target.lateTurnPoint == null) {
                double a = getDesiredAngle(Robot.currPose, target, pathPointType.isLocked());
                powerPose.heading = a;
                robot.packet.put("desired angle", a);
            } else {
                if(start.distance(Robot.currPose) > start.distance(target.lateTurnPoint)) {
                    powerPose.heading = getDesiredAngle(Robot.currPose, target, true);
                } else {
                    powerPose.heading = getDesiredAngle(Robot.currPose, target, false);
                }
            }
        } else {

            Pose relVals = Robot.currPose.relVals(finalTarget);
            Pose relLineVals = new Pose(start, start.minus(finalTarget).atan()).relVals(new Pose(finalTarget, 0));
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
        }


        System.out.println("currpose: " + Robot.currPose);
        System.out.println("target: " + target.toString());
        System.out.println("finalTarget: " + (finalTarget == null ? "" : finalTarget));
//        System.out.println("curr vel: " + robot.currVel);
//        System.out.println("vel hypot: " + robot.currVel.hypot());
        DriveTrain.powers.set(powerPose);
    }

    public static void followPath(Robot robot, BasePathPoint start, BasePathPoint end, ArrayList<BasePathPoint> allPoints) {
        Point clip = MathUtil.clipIntersection2(start, end, Robot.currPose);
        Point intersectPoint = MathUtil.circleLineIntersection(clip, start, end, end.followDistance);

        BasePathPoint followPoint = new BasePathPoint(end);
        followPoint.x = intersectPoint.x;
        followPoint.y = intersectPoint.y;

        goToPosition(robot, followPoint, end.isStop != null ? end : null, start);
//        oldGoToPosition(followPoint.x, followPoint.y, 0.7, Math.toRadians(90), 0.7, Math.toRadians(30), 0.6, true);
    }



    public static void oldGoToPosition(double targetX, double targetY, double moveSpeed, double prefAngle, double turnSpeed, double slowDownTurnRadians, double slowDownMovementFromTurnError, boolean stop) {
        double distance = Math.hypot(targetX - currPose.x, targetY - currPose.y);

        double absoluteAngleToTargetPoint = Math.atan2(targetY - currPose.y, targetX - currPose.x);
        double relativeAngleToTargetPoint = MathUtil.angleWrap(absoluteAngleToTargetPoint - (currPose.heading - Math.toRadians(90)));

        double relativeXToPoint = Math.cos(relativeAngleToTargetPoint) * distance;
        double relativeYToPoint = Math.sin(relativeAngleToTargetPoint) * distance;
        double relativeAbsXToPoint = Math.abs(relativeXToPoint);
        double relativeAbsYToPoint = Math.abs(relativeYToPoint);

        double v = relativeAbsXToPoint + relativeAbsYToPoint;
        double movementXPower = relativeXToPoint / v;
        double movementYPower = relativeYToPoint / v;

        if(stop) {
            movementXPower *= relativeAbsXToPoint / 12;
            movementYPower *= relativeAbsYToPoint / 12;
        }

        powers.x = Range.clip(movementXPower, -moveSpeed, moveSpeed);
        powers.y = Range.clip(movementYPower, -moveSpeed, moveSpeed);



        // turning and smoothing shit
        double relativeTurnAngle = prefAngle - Math.toRadians(90);
        double absolutePointAngle = absoluteAngleToTargetPoint + relativeTurnAngle;
        double relativePointAngle = MathUtil.angleWrap(absolutePointAngle - currPose.heading);

        double decelerateAngle = Math.toRadians(40);

        double movementTurnSpeed = (relativePointAngle/decelerateAngle) * turnSpeed;

        powers.heading = Range.clip(movementTurnSpeed, -turnSpeed, turnSpeed);

        if(distance < 3) {
            powers.heading = 0;
        }

        allComponentsMinPower();


        // smoothing
        powers.x *= Range.clip((relativeAbsXToPoint/3.0),0,1);
        powers.y *= Range.clip((relativeAbsYToPoint/3.0),0,1);
        powers.heading *= Range.clip(Math.abs(relativePointAngle)/Math.toRadians(2),0,1);


        //slow down if our point angle is off
        double errorTurnSoScaleDownMovement = Range.clip(1.0-Math.abs(relativePointAngle/slowDownTurnRadians),1.0-slowDownMovementFromTurnError,1);
        //don't slow down if we aren't trying to turn (distanceToPoint < 10)
        if(Math.abs(powers.heading) < 0.00001){
            errorTurnSoScaleDownMovement = 1;
        }
        powers.x *= errorTurnSoScaleDownMovement;
        powers.y *= errorTurnSoScaleDownMovement;
    }
}
