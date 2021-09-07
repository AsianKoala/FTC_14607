package org.firstinspires.ftc.teamcode.control.system

// todo
abstract class State {
    abstract val name: String
    abstract fun run()
    abstract val skipCondition: Boolean
}
