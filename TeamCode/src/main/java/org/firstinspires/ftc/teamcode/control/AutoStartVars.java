package org.firstinspires.ftc.teamcode.control;

import org.firstinspires.ftc.teamcode.util.Pose;

import static org.firstinspires.ftc.teamcode.movement.Odometry.currentPosition;

class AutoStartVars {
    public double stageStartX;
    public double stageStartY;
    public double stageStartHeading;
    public Pose stageStartPose;
    public double stageStartTime;

    public void initialize() {
        stageStartX = currentPosition.x;
        stageStartY = currentPosition.y;
        stageStartHeading = currentPosition.heading;
        stageStartPose = new Pose(stageStartX, stageStartY, stageStartHeading);
        stageStartTime = System.currentTimeMillis();
    }
}
