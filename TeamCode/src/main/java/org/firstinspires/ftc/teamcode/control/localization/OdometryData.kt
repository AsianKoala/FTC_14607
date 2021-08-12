package org.firstinspires.ftc.teamcode.control.localization

import org.firstinspires.ftc.teamcode.util.Point
import org.firstinspires.ftc.teamcode.util.Pose

data class OdometryData(val currentPosition: Pose, val robotSpeed: Point)
