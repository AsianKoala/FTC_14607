
import org.firstinspires.ftc.teamcode.control.system.State
import org.firstinspires.ftc.teamcode.control.system.StateMachineBuilder
import kotlin.jvm.JvmStatic

object KotlinTest {
    fun t(j: Long) = System.currentTimeMillis() - j < 5000

    @JvmStatic
    fun main(args: Array<String>) {
        val start = System.currentTimeMillis()

        val stateMachine = StateMachineBuilder()
            .addState(object : State() {
                override fun run() {
                    println(name)
                }

                override val name: String
                    get() = "first one"
            })
            .addState(object : State() {
                override fun run() {
                    println(name)
                }

                override val name: String
                    get() = "second one"
            })
            .build()

        while (!stateMachine.killCond()) {
            stateMachine.run()
        }
    }
}
