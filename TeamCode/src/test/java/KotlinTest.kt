
import org.firstinspires.ftc.teamcode.control.path.OnlyTurnPoint
import org.firstinspires.ftc.teamcode.control.path.PathPoint
import org.firstinspires.ftc.teamcode.util.Angle
import kotlin.jvm.JvmStatic

object KotlinTest {
    @JvmStatic
    fun main(args: Array<String>) {
        val otPoint = OnlyTurnPoint("otpoint", 1.0, 2.0, 3.0, Angle(Angle.Unit.RAW), Angle(3.0, Angle.Unit.RAD))
        val pPoint: PathPoint = otPoint.copy
        val castPoint: OnlyTurnPoint = pPoint.copy as OnlyTurnPoint
        println(pPoint is OnlyTurnPoint)
        println(castPoint.dh)
    }
}
