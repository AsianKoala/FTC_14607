package org.firstinspires.ftc.teamcode.control.localization;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.DecompositionSolver;
import org.apache.commons.math3.linear.LUDecomposition;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.firstinspires.ftc.teamcode.util.OpModeClock;
import org.firstinspires.ftc.teamcode.util.Pose;
import org.firstinspires.ftc.teamcode.util.SignaturePose;

import java.util.ArrayList;

// used to get accurate wheel deltas for odom implementations
public abstract class BaseOdometry {
    public static final double TICKS_PER_INCH = 1103.8839;
    public static final double LATERAL_DISTANCE = 10;
    public static final double HORIZONTAL_WHEEL_OFFSET = 10;
    private final DecompositionSolver forwardSolver;
    public ArrayList<SignaturePose> deltaPosesList;
    protected Pose deltaScaledWheelPositions;
    protected Pose currentRobotPoseDelta;
    protected Pose currentRobotPosition;
    protected Pose currentRobotVelocity;
    private Pose prevWheelPositions;

    public BaseOdometry(Pose startPose) {
        setStartingPose(startPose);

        Array2DRowRealMatrix inverseMatrix = new Array2DRowRealMatrix(3, 3);

        Pose[] wheelPoses = {
                new Pose(0, LATERAL_DISTANCE / 2, 0),
                new Pose(0, -LATERAL_DISTANCE / 2, 0),
                new Pose(HORIZONTAL_WHEEL_OFFSET, 0, Math.toRadians(90))
        };

        for (int i = 0; i < 3; i++) {
            Pose currentWheelPose = wheelPoses[i];
            double x = Math.cos(currentWheelPose.heading);
            double y = Math.sin(currentWheelPose.heading);

            inverseMatrix.setEntry(i, 0, x);
            inverseMatrix.setEntry(i, 1, y);
            inverseMatrix.setEntry(i, 2,
                    currentWheelPose.x * y - currentWheelPose.y * x);
        }

        forwardSolver = new LUDecomposition(inverseMatrix).getSolver();
    }

    protected Pose calcPoseDeltas(Pose deltas) {
        RealMatrix m = MatrixUtils.createRealMatrix(new double[][]{
                new double[]{
                        deltas.x,
                        deltas.y,
                        deltas.heading}});
        RealMatrix rawPoseDelta = forwardSolver.solve(m.transpose());

        return new Pose(
                rawPoseDelta.getEntry(0, 0),
                rawPoseDelta.getEntry(1, 0),
                rawPoseDelta.getEntry(2, 0)
        );
    }

    // should only be used for start of auto
    public void setStartingPose(Pose startPose) {
        currentRobotPosition = startPose;
        prevWheelPositions = new Pose();
    }

    protected abstract void robotPoseUpdate();

    public Pose[] update(Pose wheelPositions, Pose wheelVel) {
        deltaScaledWheelPositions = wheelPositions.subtract(prevWheelPositions).divide(TICKS_PER_INCH);
        currentRobotPoseDelta = calcPoseDeltas(deltaScaledWheelPositions);

        deltaPosesList.add(new SignaturePose(currentRobotPoseDelta.x, currentRobotPoseDelta.y, currentRobotPoseDelta.heading, OpModeClock.getElapsedStartTime()));

        robotPoseUpdate();
        currentRobotPosition.wrap();
        currentRobotVelocity = calcPoseDeltas(wheelVel);
        prevWheelPositions = new Pose(currentRobotPosition);
        return new Pose[]{currentRobotPosition, currentRobotVelocity, currentRobotPoseDelta};
    }
}






