package org.firstinspires.ftc.teamcode.main.control;

import org.firstinspires.ftc.teamcode.main.util.Pose;

import static org.firstinspires.ftc.teamcode.main.movement.Odometry.currentPosition;

class StageStartVars {
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
