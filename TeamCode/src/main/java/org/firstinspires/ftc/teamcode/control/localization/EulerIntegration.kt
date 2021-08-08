package org.firstinspires.ftc.teamcode.control.localization

@Deprecated("never coming back to this piece of shit")
class EulerIntegration() : BaseOdometry() {
//    override fun robotPoseUpdate() {
//        val dtheta: Angle = currPoseDelta.h
//        val sineTerm: Double
//        val cosTerm: Double
//        if (epsilonEquals(dtheta.rad, 0.0)) {
//            sineTerm = 1.0 - dtheta.rad * dtheta.rad / 6.0
//            cosTerm = dtheta.rad / 2.0
//        } else {
//            sineTerm = dtheta.sin / dtheta.rad
//            cosTerm = (1 - dtheta.cos) / dtheta.rad
//        }
//        val fieldTranslationDelta = Point(
//            sineTerm * currPoseDelta.x - cosTerm * currPoseDelta.y,
//            cosTerm * currPoseDelta.x + sineTerm * currPoseDelta.y
//        )
//        val fieldPoseDelta = Pose(fieldTranslationDelta.rotate(currPose.h), dtheta)
//        currPose.p += fieldPoseDelta.p
//        currPose.h += dtheta
//    }
}
