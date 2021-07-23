package org.firstinspires.ftc.teamcode.control.controllers;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.control.system.Robot;
import org.firstinspires.ftc.teamcode.hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.Point;
import org.firstinspires.ftc.teamcode.util.Pose;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.control.path.PathPoints.*;
import org.firstinspires.ftc.teamcode.hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.util.MathUtil;

public class PurePursuitController {
    private static final Pose mins = new Pose(0.11, 0.09, 0.11);

    public static boolean runFuncList(BasePathPoint target) {
        target.functions.removeIf(f -> f.cond() && f.func());
        return target.functions.size() == 0;
    }

    public static void goToPosition(Robot robot, BasePathPoint target, BasePathPoint start) {
        Pose powerPose = new Pose();
        boolean isStop = target.isStop;

        Pose relVals = Robot.currPose.relVals(target);
        Pose relLineVals = isStop ? new Pose(start, start.minus(target).atan()).relVals(new Pose(target, 0)):new Pose();
        relLineVals.set(relLineVals.abs());

        robot.packet.put("relX", relVals.x);
        robot.packet.put("relY", relVals.y);

        double smoothinx = relLineVals.x * 0.8 > 12 && isStop ? relLineVals.x * 0.8 : 12;
        double smoothiny = relLineVals.y * 0.8 > 12 && isStop ? relLineVals.y * 0.8 : 12;

        double v = relVals.abs().x + relVals.abs().y;
        powerPose.x = relVals.abs().x / smoothinx;
        powerPose.y = relVals.abs().y / smoothiny;
        powerPose.x *= relVals.x / v;
        powerPose.y *= relVals.y / v;

        double relAngle = getDesiredAngle(Robot.currPose, target);

        if(target.lateTurnPoint == null) {
            powerPose.h = relAngle;
        } else if(start.distance(Robot.currPose) > start.distance(target.lateTurnPoint)) {
            powerPose.h = MathUtil.angleWrap(target.lockedHeading - Robot.currPose.h);
        }
        powerPose.h /= Math.toRadians(40);
        robot.packet.put("desired angle", relAngle);


        // checks and further smoothings
        boolean turning = true;
        if(relVals.hypot() < 3) {
            powerPose.h = 0;
            turning = false;
        }

        powerPose.set(powerPose.minify(mins));
        powerPose.multiply(new Pose(
                Range.clip(relVals.x/3.0,0,1),
                Range.clip(relVals.y/3.0,0,1),
                Range.clip(Math.abs(powerPose.h)/Math.toRadians(2),0,1)));

        //TODO experiment with min relAngle for scaling
        //TODO experiment with max scalar value
        double turnErrorScaler = turning?Range.clip(1.0-Math.abs(relAngle/Math.toRadians(40)),0.4,1):1;
        powerPose.x *= turnErrorScaler;
        powerPose.y *= turnErrorScaler;

        if(target.isStop) {
            Point extend = MathUtil.extendLine(start, target, 30); //todo experiment with extend line lookahead point
            double newTargetAngle = extend.minus(Robot.currPose).atan();

            double dH = MathUtil.angleWrap(newTargetAngle-Robot.currPose.h);
            DriveTrain.powers.h = relAngle / Math.toRadians(45);
            DriveTrain.powers.minify(mins);
            DriveTrain.powers.h *= Range.clip(Math.abs(relAngle)/Math.toRadians(3),0,1);

            double stopTurnErrorScalar = Math.abs(dH) / Math.toRadians(40);
            DriveTrain.powers.x *= 1 - Range.clip(stopTurnErrorScalar,0,0.6);
            DriveTrain.powers.y *= 1 - Range.clip(stopTurnErrorScalar,0,0.6);
        }

        DriveTrain.powers.set(powerPose);
    }

    private static double getDesiredAngle(Pose curr, BasePathPoint target) {
        double forward = target.minus(curr).atan();
        double back = forward + Math.PI;
        double angleToForward = MathUtil.angleWrap(forward - curr.h);
        double angleToBack = MathUtil.angleWrap(back - curr.h);
        double autoAngle = Math.abs(angleToForward) < Math.abs(angleToBack) ? forward : back;
        return MathUtil.angleWrap(autoAngle - curr.h);
    }

    public static void followPath(Robot robot, BasePathPoint start, BasePathPoint end) {
        Point clip = MathUtil.clipIntersection2(start, end, Robot.currPose);
        Point intersectPoint = MathUtil.circleLineIntersection(clip, start, end, end.followDistance);

        BasePathPoint followPoint = new BasePathPoint(end);
        followPoint.x = intersectPoint.x;
        followPoint.y = intersectPoint.y;

        goToPosition(robot, followPoint, start);
    }
}
