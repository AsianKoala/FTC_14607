
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
        val x = -10.032038777984924
        val y = 1.1895285941307732
        val followx = 11.54
        val followy = -2.30
        val curr = Point(x, y)
        val target = Point(followx, followy)
        val d = (curr - target).hypot

        println(d)

        //         val rh = (target.p - curr.p).atan2 - curr.h
        //        return Point(-d * rh.sin, d * rh.cos)

        val rh = ((target - curr).atan2.rad - ((-120.36).toRadians)).wrap

        println(rh)

        val final = Point(-d * sin(rh), d * cos(rh))

        println(final / (final.x.absoluteValue + final.y.absoluteValue))
    }
}
