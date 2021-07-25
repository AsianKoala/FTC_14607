package org.firstinspires.ftc.teamcode.control.localization;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.DecompositionSolver;
import org.apache.commons.math3.linear.LUDecomposition;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.firstinspires.ftc.teamcode.control.system.Robot;
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.Point;
import org.firstinspires.ftc.teamcode.util.Pose;
import org.openftc.revextensions2.RevBulkData;
import static org.firstinspires.ftc.teamcode.control.system.Robot.*;

public class TwoWheelTrackingLocalizer {
    public static double PARALLEL_TICKS_PER_INCH = 1103.8839;
    public static double PERP_TICKS_PER_INCH = 1103.8839;

    private final static double PARALLEL_X = 6.75;
    private final static double PARALLEL_Y = 4.25;

    private final static double PERPENDICULAR_X = 6.75;
    private final static double PERPENDICULAR_Y = -4.25;

    DecompositionSolver forwardSolver;

    int[] prevWheelPositions;
    double prevHeading;

    // External interfaces
    private Pose currentPosition;
    private Pose relativeRobotMovement;

    public TwoWheelTrackingLocalizer() {
        this(new Pose(0,0,0));
    }

    public TwoWheelTrackingLocalizer(Pose start) {
        Array2DRowRealMatrix inverseMatrix = new Array2DRowRealMatrix(3, 3);

        EncoderWheel[] WHEELS = {
                new EncoderWheel(PARALLEL_X, PARALLEL_Y, 0, 0), // parallel
                new EncoderWheel(PERPENDICULAR_X, PERPENDICULAR_Y, Math.toRadians(90), 1), // perp todo swithc heading if fail
        };

        for (EncoderWheel wheelPosition : WHEELS) {
            double x = Math.cos(wheelPosition.heading);
            double y = Math.sin(wheelPosition.heading);

            inverseMatrix.setEntry(wheelPosition.row, 0, x);
            inverseMatrix.setEntry(wheelPosition.row, 1, y);
            inverseMatrix.setEntry(wheelPosition.row, 2,
                    wheelPosition.x * y - wheelPosition.y * x);
        }
        inverseMatrix.setEntry(2, 2, 1.0);

        forwardSolver = new LUDecomposition(inverseMatrix).getSolver();

        if (!forwardSolver.isNonSingular()) {
            throw new IllegalArgumentException("The specified configuration cannot support full localization");
        }

        prevWheelPositions = new int[2]; // Initializes with zeros

        currentPosition = new Pose(start.x, start.y, start.h);
        relativeRobotMovement = new Pose(0, 0, 0);
    }

    public static double encoderTicksToInches(int ticks, double ticksPerInch) {
        return ticks / ticksPerInch;
    }

    public static int inchesToEncoderTicks(double inches) {
        return (int) (inches * PARALLEL_TICKS_PER_INCH);
    }

    public void update(RevBulkData data, double heading) {
        double[] deltas = new double[] {
                encoderTicksToInches(data.getMotorCurrentPosition(PARALLEL_ENCODER_PORT) - prevWheelPositions[0],
                        PARALLEL_TICKS_PER_INCH),
                encoderTicksToInches(data.getMotorCurrentPosition(PERP_ENCODER_PORT) - prevWheelPositions[1],
                        PERP_TICKS_PER_INCH),
                MathUtil.angleWrap(heading - prevHeading)
        };
        prevWheelPositions[0] = data.getMotorCurrentPosition(PARALLEL_ENCODER_PORT);
        prevHeading = heading;
        prevWheelPositions[1] = data.getMotorCurrentPosition(PERP_ENCODER_PORT);
        updateFromRelative(deltas);
    }

    public void updateFromRelative(double[] deltas) {
        RealMatrix m = MatrixUtils.createRealMatrix(new double[][] {deltas});

        RealMatrix rawPoseDelta = forwardSolver.solve(m.transpose());
        Pose robotPoseDelta = new Pose(
                rawPoseDelta.getEntry(0, 0),
                rawPoseDelta.getEntry(1, 0),
                rawPoseDelta.getEntry(2, 0)
        );

        relativeRobotMovement = relativeRobotMovement.add(robotPoseDelta);
        currentPosition = relativeOdometryUpdate(currentPosition, robotPoseDelta);
    }

    private Pose relativeOdometryUpdate(Pose fieldPose, Pose robotPoseDelta) {
        double dtheta = robotPoseDelta.h;
        double sineTerm, cosTerm;

        if (MathUtil.epsilonEquals(dtheta, 0)) {
            sineTerm = 1.0 - dtheta * dtheta / 6.0;
            cosTerm = dtheta / 2.0;
        } else {
            sineTerm = Math.sin(dtheta) / dtheta;
            cosTerm = (1 - Math.cos(dtheta)) / dtheta;
        }

        Point fieldPositionDelta = new Point(
                sineTerm * robotPoseDelta.x + cosTerm * robotPoseDelta.y,
                cosTerm * robotPoseDelta.x + sineTerm * robotPoseDelta.y
        );

        Pose fieldPoseDelta = new Pose(fieldPositionDelta.rotated(fieldPose.h), robotPoseDelta.h);

        return fieldPose.add(fieldPoseDelta);
    }

    public Pose getPoseUpdate() {
        return currentPosition;
    }
}
