package org.firstinspires.ftc.teamcode.control.localization

@Deprecated("never coming back to this piece of shit")
class ExactArcOdometry : BaseOdometry() {
//    private static final double TRACK_WIDTH = 12.0;
//    private static final double PARALLEL_WIDTH = 10.0;
//
//    public ExactArcOdometry(Pose startPose) {
//        super(startPose);
//    }
//
//    @Override
//    protected void robotPoseUpdate() {
//        double dtheta = currPoseDelta.h;
//        double currHeading = MathUtil.angleWrap(dtheta + currPose.h);
//        double dx, dy;
//
//        if (MathUtil.epsilonEquals(dtheta, 0)) {
//            dx = wheelDeltaScaled.h;
//            dy = (wheelDeltaScaled.x + wheelDeltaScaled.y) / 2;
//        } else {
//            double turnRadius = (TRACK_WIDTH / 2) * (wheelDeltaScaled.x + wheelDeltaScaled.y) / (wheelDeltaScaled.x - wheelDeltaScaled.y);
//            double strafeRadius = wheelDeltaScaled.h / dtheta - PARALLEL_WIDTH;
//
//            dx = turnRadius * (currPoseDelta.cos() - 1) + strafeRadius * currPoseDelta.sin();
//            dy = turnRadius * currPoseDelta.sin() + strafeRadius * (1 - currPoseDelta.cos());
//        }
//
//        Point fieldTranslationDelta = new Point(dx, dy);
//        Pose fieldPoseDelta = new Pose(fieldTranslationDelta.rotated(currHeading), dtheta);
//
//        currPose = currPose.add(fieldPoseDelta);
//    }
}