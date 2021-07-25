package org.firstinspires.ftc.teamcode.control.localization;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.DecompositionSolver;
import org.apache.commons.math3.linear.LUDecomposition;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.firstinspires.ftc.teamcode.util.Pose;
import org.firstinspires.ftc.teamcode.util.SignaturePose;

import java.util.ArrayList;

// used to get accurate wheel deltas for odom implementations
public abstract class BaseOdometry {
    private static final double TICKS_PER_INCH = 1103.8839;
    private static final double LATERAL_DISTANCE = 10;
    private static final double HORIZONTAL_WHEEL_OFFSET = 10;


    protected Pose wheelDeltaScaled;
    protected Pose currPoseDelta;
    protected Pose currPose;

    private Pose currVelocity;
    private Pose prevWheelPositions;
    private final ArrayList<SignaturePose> prevPoses;
    private Pose robotPoseDelta;

    private final DecompositionSolver forwardSolver;

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
            double x = Math.cos(currentWheelPose.h);
            double y = Math.sin(currentWheelPose.h);

            inverseMatrix.setEntry(i, 0, x);
            inverseMatrix.setEntry(i, 1, y);
            inverseMatrix.setEntry(i, 2,
                    currentWheelPose.x * y - currentWheelPose.y * x);
        }

        forwardSolver = new LUDecomposition(inverseMatrix).getSolver();
        robotPoseDelta = new Pose(0,0,0);
        prevPoses = new ArrayList<>();
        prevPoses.add(new SignaturePose(robotPoseDelta, System.currentTimeMillis()));
    }

    protected Pose calcPoseDeltas(Pose deltas) {
        RealMatrix m = MatrixUtils.createRealMatrix(new double[][]{
                new double[]{
                        deltas.x,
                        deltas.y,
                        deltas.h}});
        RealMatrix rawPoseDelta = forwardSolver.solve(m.transpose());

        return new Pose(
                rawPoseDelta.getEntry(0, 0),
                rawPoseDelta.getEntry(1, 0),
                rawPoseDelta.getEntry(2, 0)
        );
    }

    private void calcRobotVel() {
        if (prevPoses.size() > 1) {

            int oldIndex = Math.max(0, prevPoses.size() - 2 - 1);
            SignaturePose old = prevPoses.get(oldIndex);
            SignaturePose cur = prevPoses.get(prevPoses.size() - 1);
            double scale = (double) (cur.sign - old.sign) / (1000);
            currVelocity = new Pose(cur.minus(old).scale(1 / scale));
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

    public Pose[] update(Pose wheelPositions) { // todo fix vel
        wheelPositions.minus(prevWheelPositions);
        wheelDeltaScaled = wheelPositions.scale(TICKS_PER_INCH);

        currPoseDelta = calcPoseDeltas(wheelDeltaScaled);

        Pose oldPose = currPose;
        robotPoseUpdate();
        currPose = currPose.wrap();

        Pose deltaVals = currPose.minus(oldPose);
        robotPoseDelta = new Pose(robotPoseDelta.add(deltaVals));

        prevPoses.add(new SignaturePose(currPose, System.currentTimeMillis()));
        calcRobotVel();

        prevWheelPositions = new Pose(currPose);
        return new Pose[]{currPose, currVelocity, currPoseDelta};
    }
}






