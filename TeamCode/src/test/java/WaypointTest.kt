import org.firstinspires.ftc.teamcode.control.path.waypoints.StopWaypoint
import org.firstinspires.ftc.teamcode.control.path.waypoints.Waypoint
import org.firstinspires.ftc.teamcode.util.math.Angle
import org.firstinspires.ftc.teamcode.util.math.AngleUnit
import org.firstinspires.ftc.teamcode.util.math.MathUtil

object WaypointTest {
    @JvmStatic
    fun main(args: Array<String>) {
        val curr = Waypoint(38.0, 58.0, 0.0)
        val start = Waypoint(38.0, 58.0, 0.0)
        val end = StopWaypoint(15.0, 54.0, 13.0, Angle(0.0, AngleUnit.RAD))
        val clip = MathUtil.clipIntersection(start.p, end.p, curr.p)
        val (x, y) = MathUtil.circleLineIntersection(clip, start.p, end.p, end.followDistance)
        val followPoint = end.copy
        followPoint.x = x
        followPoint.y = y

        println("$x, $y")

        println(followPoint.toString())
    }
}