package org.firstinspires.ftc.teamcode.opmodes

import org.firstinspires.ftc.teamcode.control.system.BaseOpMode
import org.firstinspires.ftc.teamcode.util.math.Angle
import org.firstinspires.ftc.teamcode.util.math.AngleUnit
import org.firstinspires.ftc.teamcode.util.math.Point
import org.firstinspires.ftc.teamcode.util.math.Pose

class FireflyTele: BaseOpMode() {
    override val startPose: Pose
        get() = Pose(Point.ORIGIN, Angle(0.0, AngleUnit.RAD))

    override fun onLoop() {
        firefly.teleopControl(0.7)
    }
}