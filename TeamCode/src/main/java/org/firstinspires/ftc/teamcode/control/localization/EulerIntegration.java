package org.firstinspires.ftc.teamcode.control.localization;

import org.firstinspires.ftc.teamcode.util.Point;
import org.firstinspires.ftc.teamcode.util.Pose;

import static org.firstinspires.ftc.teamcode.util.MathUtil.epsilonEquals;

public class EulerIntegration extends BaseOdometry {

    public EulerIntegration(Pose startPose) {
        super(startPose);
    }

    @Override
    protected void robotPoseUpdate() {
        double dtheta = currPoseDelta.heading;
        double sineTerm, cosTerm;

        if (epsilonEquals(dtheta, 0)) {
            sineTerm = 1.0 - dtheta * dtheta / 6.0;
            cosTerm = dtheta / 2.0;
        } else {
            sineTerm = Math.sin(dtheta) / dtheta;
            cosTerm = (1 - Math.cos(dtheta)) / dtheta;
        }

        Point fieldTranslationDelta = new Point(
                sineTerm * currPoseDelta.x - cosTerm * currPoseDelta.y,
                cosTerm * currPoseDelta.x + sineTerm * currPoseDelta.y
        );

        Pose fieldPoseDelta = new Pose(fieldTranslationDelta.rotated(currPose.heading), dtheta);
        currPose.add(fieldPoseDelta);
    }
}
