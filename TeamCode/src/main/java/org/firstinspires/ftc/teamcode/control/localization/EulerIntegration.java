package org.firstinspires.ftc.teamcode.control.localization;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.DecompositionSolver;
import org.apache.commons.math3.linear.LUDecomposition;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.firstinspires.ftc.teamcode.util.Point;
import org.firstinspires.ftc.teamcode.util.Pose;

import static org.firstinspires.ftc.teamcode.util.MathUtil.epsilonEquals;

public class EulerIntegration extends BaseOdometry {

    private final DecompositionSolver forwardSolver;

    public EulerIntegration(Pose startPose) {
        super(startPose);

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

    private Pose calcPoseDeltas(Pose deltas) {
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

    @Override
    protected void robotPoseUpdate() {
        Pose robotPoseDelta = calcPoseDeltas(deltaScaledWheelPositions);

        // TODO need to check if the code above is accurate enough to be used in the other odometry subclasses
        double dtheta = robotPoseDelta.heading;
        double sineTerm, cosTerm;

        if (epsilonEquals(dtheta, 0)) {
            sineTerm = 1.0 - dtheta * dtheta / 6.0;
            cosTerm = dtheta / 2.0;
        } else {
            sineTerm = Math.sin(dtheta) / dtheta;
            cosTerm = (1 - Math.cos(dtheta)) / dtheta;
        }

        Point fieldTranslationDelta = new Point(
                sineTerm * robotPoseDelta.x - cosTerm * robotPoseDelta.y,
                cosTerm * robotPoseDelta.x + sineTerm * robotPoseDelta.y
        );

        Pose fieldPoseDelta = new Pose(fieldTranslationDelta.rotated(currentRobotPosition.heading), robotPoseDelta.heading);
        currentRobotPosition.add(fieldPoseDelta);

        currentRobotVelocity = calcPoseDeltas(currentWheelVelocity);
    }
}
