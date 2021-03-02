package org.firstinspires.ftc.teamcode.control.controllers;

import org.firstinspires.ftc.teamcode.control.path.PathPoint;
import org.firstinspires.ftc.teamcode.robot.Robot;

public class PurePursuitController {

    public static void goToPosition(Robot robot, PathPoint target) {

    }

    public static void followPath(Robot robot) {
        if(!robot.pathCache.isPurePursuit) {
            boolean finished = robot.currPose.poseClose(robot.pathCache.getFirst());
            if(finished)
                robot.pathCache.removeFirst();
            else
                goToPosition(robot, robot.pathCache.getFirst());
        }
    }
}
