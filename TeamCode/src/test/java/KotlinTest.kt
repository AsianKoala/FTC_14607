
object KotlinTest {
    fun t(j: Long) = System.currentTimeMillis() - j < 5000

    fun interface marker {
        fun onreached()
    }

    val l = ArrayList<tempmarker>()

    data class tempmarker(val t: Double, val callback: marker)

    fun addmarker(time: Double, callback: marker) {
        l.add(tempmarker(time, callback))
    }

    @JvmStatic
    fun main(args: Array<String>) {
        addmarker(2.0) {
            println("hello")
        }

        addmarker(3.0) {
            println("asdadasd")
        }

        l.forEach {
            println("time: ${it.t}")
            it.callback.onreached()
        }

//        val start = System.currentTimeMillis()
//
//        val stateMachine = StateMachineBuilder()
//            .addState(object : State() {
//                override fun run() {
//                    println(name)
//                }
//
//                override val name: String
//                    get() = "first one"
//            })
//            .addState(object : State() {
//                override fun run() {
//                    println(name)
//                }
//
//                override val name: String
//                    get() = "second one"
//            })
//            .build()
//
//        while (!stateMachine.killCond()) {
//            stateMachine.run()
//        }
    }
}
