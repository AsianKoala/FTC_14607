import org.firstinspires.ftc.teamcode.control.path.waypoints.StopWaypoint
import org.firstinspires.ftc.teamcode.control.path.waypoints.Waypoint
import org.firstinspires.ftc.teamcode.util.math.Angle
import org.firstinspires.ftc.teamcode.util.math.AngleUnit
import org.firstinspires.ftc.teamcode.util.math.MathUtil
import org.firstinspires.ftc.teamcode.util.math.MathUtil.epsilonEquals
import org.firstinspires.ftc.teamcode.util.math.Point

object WaypointTest {
    @JvmStatic
    fun main(args: Array<String>) {
        val curr = Waypoint(38.0, 58.0, 0.0)
        val start = Waypoint(38.0, 58.0, 0.0)
        val end = StopWaypoint(15.0, 54.0, 10.0, Angle(0.0, AngleUnit.RAD))
        val clip: Point = MathUtil.clipIntersection(start.p, end.p, curr.p)
        val (x, y) = MathUtil.circleLineIntersection(clip, start.p, end.p, end.followDistance)
        val followPoint = end.copy
        followPoint.x = x
        followPoint.y = y

        println("${followPoint.x}, ${followPoint.y}")

        println(followPoint)
        println(followPoint.p)
    }
}
