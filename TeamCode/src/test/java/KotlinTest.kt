
import org.firstinspires.ftc.teamcode.util.math.Point
import kotlin.jvm.JvmStatic

object KotlinTest {
    @JvmStatic
    fun main(args: Array<String>) {
        val foo = Point(0.0, 0.0)
        val bar = foo
        bar.x = 10.0
        println(foo)
        println(bar)
    }
}
