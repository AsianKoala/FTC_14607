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
    private static final double TICKS_PER_INCH = 1103.8839;
    private static final double LATERAL_DISTANCE = 10;
    private static final double HORIZONTAL_WHEEL_OFFSET = 10;
    private final DecompositionSolver forwardSolver;
    private ArrayList<SignaturePose> relativePoseDeltas;
    protected Pose wheelDeltaScaled;
    protected Pose currPoseDelta;
    protected Pose currPose;
    private Pose currVelocity;
    private Pose relativePoseDelta;
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

    private void calcRobotVel() {
        if (relativePoseDeltas.size() > 1) {
            SignaturePose oldPos = relativePoseDeltas.get(relativePoseDeltas.size() - 2);
            SignaturePose newPos = relativePoseDeltas.get(relativePoseDeltas.size() - 1);
            double timeDiff = (newPos.signature - oldPos.signature) / 1000.0;
            currVelocity = newPos.subtract(oldPos).multiply(1 / timeDiff);
        } else {
            currVelocity = new Pose(0, 0, 0);
        }
    }

    // should only be used for start of auto
    public void setStartingPose(Pose startPose) {
        currPose = startPose;
        prevWheelPositions = new Pose();
    }

    protected abstract void robotPoseUpdate();

    public Pose[] update(Pose wheelPositions, Pose wheelVel) {
        wheelDeltaScaled = wheelPositions.subtract(prevWheelPositions).divide(TICKS_PER_INCH);
        currPoseDelta = calcPoseDeltas(wheelDeltaScaled);

        relativePoseDelta = relativePoseDelta.add(currPoseDelta);
        relativePoseDeltas.add(new SignaturePose(relativePoseDelta.x, relativePoseDelta.y, relativePoseDelta.heading, OpModeClock.getElapsedStartTime()));

        robotPoseUpdate();
        currPose.wrap();
        calcRobotVel();
        prevWheelPositions = new Pose(currPose);
        return new Pose[]{currPose, currVelocity, currPoseDelta};
    }
}






