package org.firstinspires.ftc.teamcode.control.localization;

import org.firstinspires.ftc.teamcode.util.OpModeClock;
import org.firstinspires.ftc.teamcode.util.Pose;
import org.firstinspires.ftc.teamcode.util.SignaturePose;

import java.util.ArrayList;

// used to get accurate wheel deltas for odom implementations
public abstract class BaseOdometry {
    public static final double TICKS_PER_INCH = 1103.8839;
    public static final double LATERAL_DISTANCE = 10;
    public static final double HORIZONTAL_WHEEL_OFFSET = 10;
    public ArrayList<SignaturePose> deltaPosesList;

    // honestly theres a lot of redundancy in this and it can be simplified (basically all of the redundant Poses) ut im just leaving it as it is for now just in case i want to do something with the values
    // always go left right horiz
    protected Pose startPose;
    // used in subclasses
    protected Pose currentWheelVelocity;
    protected Pose deltaScaledWheelPositions;
    protected Pose currentRobotPosition;
    protected Pose currentRobotVelocity;
    private Pose currentWheelPositions;
    private Pose prevWheelPositions;

    public BaseOdometry(Pose startPose) {
        setStartingPose(startPose);
    }

    // should only be used for start of auto
    public void setStartingPose(Pose startPose) {
        this.startPose = startPose;
        currentRobotPosition = startPose;
        currentRobotVelocity = new Pose();
        currentWheelPositions = new Pose();
        prevWheelPositions = new Pose();
        currentWheelVelocity = new Pose();
    }

    protected abstract void robotPoseUpdate();

    public Pose[] update(Pose wheelPositions, Pose wheelVel) {
        currentWheelPositions = wheelPositions;
        currentWheelVelocity = wheelVel;

        deltaScaledWheelPositions = currentWheelPositions.subtract(prevWheelPositions).divide(TICKS_PER_INCH);
        deltaPosesList.add(new SignaturePose(deltaScaledWheelPositions.x, deltaScaledWheelPositions.y, deltaScaledWheelPositions.heading, OpModeClock.getElapsedStartTime()));

        robotPoseUpdate();
        currentRobotPosition.wrap();
        return new Pose[]{currentRobotPosition, currentRobotVelocity};
    }
}






