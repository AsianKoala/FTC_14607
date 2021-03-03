package org.firstinspires.ftc.teamcode.control.controllers;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.control.path.PathPoint;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.Pose;

public class PurePursuitController {
    public static Pose max_speed = new Pose(1, 1, 1); // xd
    public static Pose stop_speed = new Pose(0.1, 0.1, 0.1);

    public static void goToPosition(Robot robot, PathPoint target) {
        Pose relVals = robot.currPose.distancePose(target);

        double v = relVals.abs().x + relVals.abs().y;
        Pose powerPose = new Pose(relVals.x, relVals.y, 69420); // heading doesnt matter for now so ya
        powerPose.divide(new Pose(v));
        powerPose.multiply(relVals.abs().divide(new Pose(12)));

        powerPose.heading = dH(robot, target) / Math.toRadians(40);
        powerPose.clipMax(max_speed);

        powerPose.multiply(new Pose(Range.clip(1.0-Math.abs(dH(robot,target) / Math.toRadians(40)), 0.6, 1)));

        robot.currPowers.set(powerPose);

//        if(robot.currPose.distance(target) < 6 && target.stop) {
//            relVals = robot.currPose.distancePose(target);
//            Pose absMult = relVals.abs().divide(new Pose(6));
//            robot.currPowers.set(relVals.multiply(absMult));
//        }wwwwwwwwwwww

    }

    private static double dH(Robot robot, PathPoint target) {
        return target.locked ? MathUtil.angleWrap(target.heading - robot.currPose.heading) : MathUtil.angleWrap(robot.currPose.subtract(target).atan() - robot.currPose.heading);
    }


    public static void followPath(Robot robot) {
//        if(!robot.pathCache.isPurePursuit) {
//            boolean finished = robot.currPose.poseClose(robot.pathCache.getFirst());
//            if(finished)
//                robot.pathCache.removeFirst();
//            else
//                goToPosition(robot, robot.pathCache.getFirst());
//        }
    }
}
