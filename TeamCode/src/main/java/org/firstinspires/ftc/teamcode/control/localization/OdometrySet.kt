package org.firstinspires.ftc.teamcode.control.localization

import org.openftc.revextensions2.ExpansionHubMotor

data class OdometrySet(
    private val vertical: ExpansionHubMotor,
    private val horizontal: ExpansionHubMotor
) {
    private var verticalOffset = 0
    private var horizontalOffset = 0
    private fun markCurrOffset() {
        verticalOffset = vertical.currentPosition
        horizontalOffset = horizontal.currentPosition
    }

    val verticalTicks: Int
        get() = vertical.currentPosition - verticalOffset
    val horizontalTicks: Int
        get() = horizontal.currentPosition - horizontalOffset

    init {
        markCurrOffset()
    }
}
