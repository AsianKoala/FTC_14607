
import org.firstinspires.ftc.teamcode.util.Point
import kotlin.jvm.JvmStatic

object KotlinTest {
    @JvmStatic
    fun main(args: Array<String>) {
        var o = Point()
        o.x += 100
        println(o.toString())
    }
}
