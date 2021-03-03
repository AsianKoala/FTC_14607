package org.firstinspires.ftc.teamcode.control.controllers;

import org.firstinspires.ftc.teamcode.control.path.PathPoint;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.Pose;

public class PurePursuitController {
    public static Pose max_speed = new Pose(1, 1, 1); // xd

    public static void goToPosition(Robot robot, PathPoint target) {
        Pose relDistances  = robot.currPose.distancePose(target);

        double v = relDistances.abs().x + relDistances.abs().y;
        Pose powerPose = new Pose(relDistances.x, relDistances.y, 69420); // heading doesnt matter for now so ya
        powerPose.divide(new Pose(v));
        powerPose.multiply(relDistances.abs().divide(new Pose(12)));

        powerPose.heading = dH(robot, target) / Math.toRadians(45);
        robot.currPowers.set(powerPose.clipMax(max_speed));

        if(robot.currPose.distance(target) < 6 && target.stop) {
            double exp = 1.0/6.0;
            Pose speedSgns = robot.currPowers.sgns();
            Pose absSpeeds = robot.currPowers.abs();
            robot.currPowers = absSpeeds.pow(new Pose(exp)).multiply(speedSgns);
        }

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
