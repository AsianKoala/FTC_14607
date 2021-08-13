package org.firstinspires.ftc.teamcode.control.system

enum class Status {
    INIT_LOOP,
    START,
    LOOP,
    STOP;

    val next = values()[this.ordinal + 1]
}