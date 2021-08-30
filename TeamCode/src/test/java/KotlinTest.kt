
import org.firstinspires.ftc.teamcode.control.path.waypoints.LockedWaypoint
import org.firstinspires.ftc.teamcode.control.path.waypoints.Waypoint
import org.firstinspires.ftc.teamcode.util.math.Angle
import org.firstinspires.ftc.teamcode.util.math.AngleUnit
import org.firstinspires.ftc.teamcode.util.math.MathUtil.toRadians
import org.firstinspires.ftc.teamcode.util.math.MathUtil.wrap
import org.firstinspires.ftc.teamcode.util.math.Point
import kotlin.jvm.JvmStatic
import kotlin.math.absoluteValue
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin

object KotlinTest {
    @JvmStatic
    fun main(args: Array<String>) {
        val start = Waypoint(0.0, 0.0, 0.0)
        val startLocked = LockedWaypoint(0.0, 0.0, 0.0, Angle(0.0, AngleUnit.RAD))

    }
}
