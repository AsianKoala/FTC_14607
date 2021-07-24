package org.firstinspires.ftc.teamcode.control.controllers;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.control.system.Robot;
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.Point;
import org.firstinspires.ftc.teamcode.util.Pose;

import static org.firstinspires.ftc.teamcode.control.path.PathPoints.*;

public class PurePursuitController {
    private static final Pose mins = new Pose(0.11, 0.09, 0.11);

    public static void goToPosition(Robot robot, BasePathPoint target) {
        Pose relVals = robot.currPose.relVals(target);
        robot.packet.addData("relX", relVals.x);
        robot.packet.addData("relY", relVals.y);

        double smoothing = 12;
        double v = relVals.abs().x + relVals.abs().y;
        Pose powerPose = relVals.multiply(relVals).scale(1/(v * smoothing));

        double relAngle = getDesiredAngle(robot.currPose, target);
        powerPose.h = relAngle / Math.toRadians(35);
        robot.packet.addData("relH", Math.toDegrees(relAngle));

        // checks and further smoothings
        boolean turning = true;
        if(relVals.hypot() < 3) {
            powerPose.h = 0;
            turning = false;
        }

//        powerPose = powerPose.minify(mins);
        powerPose = powerPose.multiply(new Pose(
                Range.clip(relVals.x/3.0,0,1),
                Range.clip(relVals.y/3.0,0,1),
                Range.clip(Math.abs(powerPose.h)/Math.toRadians(5),0,1)));

        double turnErrorScaler = turning?Range.clip(1.0-Math.abs(relAngle/Math.toRadians(40)),0.4,0.7):1;
        robot.packet.addData("turnErrorScalar", turnErrorScaler);
        powerPose.x *= turnErrorScaler;
        powerPose.y *= turnErrorScaler;

        powerPose.x *= 1 - Range.clip(Math.abs(powerPose.h),0,0.8);
        powerPose.y *= 1 - Range.clip(Math.abs(powerPose.h),0,0.8);

        robot.driveTrain.powers = powerPose.clipAbs(1.0);
    }

    private static double getDesiredAngle(Pose curr, BasePathPoint target) {
        double forward = target.minus(curr).atan();
        double back = forward + Math.PI;
        double angleToForward = MathUtil.angleWrap(forward - curr.h);
        double angleToBack = MathUtil.angleWrap(back - curr.h);
        double autoAngle = Math.abs(angleToForward) < Math.abs(angleToBack) ? forward : back;
        return MathUtil.angleWrap(autoAngle - curr.h);
    }

    public static boolean runFuncList(BasePathPoint target) {
        target.functions.removeIf(f -> f.cond() && f.func());
        return target.functions.size() == 0;
    }

    public static void followPath(Robot robot, BasePathPoint start, BasePathPoint end) {
        Point clip = MathUtil.clipIntersection2(start, end, robot.currPose);
        Point intersectPoint = MathUtil.circleLineIntersection(clip, start, end, end.followDistance);

        BasePathPoint followPoint = new BasePathPoint(end);
        followPoint.x = intersectPoint.x;
        followPoint.y = intersectPoint.y;
        robot.packet.fieldOverlay()
                .setFill("white")
                .fillCircle(followPoint.dbNormalize().x, followPoint.dbNormalize().y,2);
        goToPosition(robot, followPoint);
    }
}
