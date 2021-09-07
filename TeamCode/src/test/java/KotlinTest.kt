
import org.firstinspires.ftc.teamcode.control.system.State
import org.firstinspires.ftc.teamcode.control.system.StateMachineBuilder
import kotlin.jvm.JvmStatic

object KotlinTest {
    @JvmStatic
    fun main(args: Array<String>) {
        val stateMachine = StateMachineBuilder()
            .addState(object : State() {
                override val name: String
                    get() = "test state"

                override fun run() {
                    println("HELLO")
                }

                override val skipCondition: Boolean
                    get() = true
            })
            .addState(object : State() {
                override val name: String
                    get() = "second state"

                override fun run() {
                    println("wow")
                }

                override val skipCondition: Boolean
                    get() = true
            }).build()

        while (stateMachine.running) {
            stateMachine.run()
        }
    }
}
