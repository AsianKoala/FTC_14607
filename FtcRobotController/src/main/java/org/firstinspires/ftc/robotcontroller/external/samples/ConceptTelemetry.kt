/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.firstinspires.ftc.robotcontroller.external.samples

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.Telemetry

/**
 * [ConceptTelemetry] illustrates various ways in which telemetry can be
 * transmitted from the robot controller to the driver station. The sample illustrates
 * numeric and text data, formatted output, and optimized evaluation of expensive-to-acquire
 * information. The telemetry [log][Telemetry.log] is illustrated by scrolling a poem
 * to the driver station.
 *
 * @see Telemetry
 */
@TeleOp(name = "Concept: Telemetry", group = "Concept")
@Disabled
class ConceptTelemetry : LinearOpMode() {
    /**
     * keeps track of the line of the poem which is to be emitted next
     */
    var poemLine = 0

    /**
     * keeps track of how long it's been since we last emitted a line of poetry
     */
    var poemElapsed = ElapsedTime()
    override fun runOpMode() {

        /* we keep track of how long it's been since the OpMode was started, just
         * to have some interesting data to show */
        val opmodeRunTime = ElapsedTime()

        // We show the log in oldest-to-newest order, as that's better for poetry
        telemetry.log().displayOrder = Telemetry.Log.DisplayOrder.OLDEST_FIRST
        // We can control the number of lines shown in the log
        telemetry.log().capacity = 6
        // The interval between lines of poetry, in seconds
        val sPoemInterval = 0.6
        /**
         * Wait until we've been given the ok to go. For something to do, we emit the
         * elapsed time as we sit here and wait. If we didn't want to do anything while
         * we waited, we would just call [.waitForStart].
         */
        while (!isStarted) {
            telemetry.addData("time", "%.1f seconds", opmodeRunTime.seconds())
            telemetry.update()
            idle()
        }

        // Ok, we've been given the ok to go
        /**
         * As an illustration, the first line on our telemetry display will display the battery voltage.
         * The idea here is that it's expensive to compute the voltage (at least for purposes of illustration)
         * so you don't want to do it unless the data is *actually* going to make it to the
         * driver station (recall that telemetry transmission is throttled to reduce bandwidth use.
         * Note that getBatteryVoltage() below returns 'Infinity' if there's no voltage sensor attached.
         *
         * @see Telemetry.getMsTransmissionInterval
         */
        telemetry.addData("voltage", "%.1f volts") { batteryVoltage }

        // Reset to keep some timing stats for the post-'start' part of the opmode
        opmodeRunTime.reset()
        var loopCount = 1

        // Go go gadget robot!
        while (opModeIsActive()) {

            // Emit poetry if it's been a while
            if (poemElapsed.seconds() > sPoemInterval) {
                emitPoemLine()
            }

            // As an illustration, show some loop timing information
            telemetry.addData("loop count", loopCount)
            telemetry.addData("ms/loop", "%.3f ms", opmodeRunTime.milliseconds() / loopCount)

            // Show joystick information as some other illustrative data
            telemetry.addLine("left joystick | ")
                .addData("x", gamepad1.left_stick_x)
                .addData("y", gamepad1.left_stick_y)
            telemetry.addLine("right joystick | ")
                .addData("x", gamepad1.right_stick_x)
                .addData("y", gamepad1.right_stick_y)
            /**
             * Transmit the telemetry to the driver station, subject to throttling.
             * @see Telemetry.getMsTransmissionInterval
             */
            telemetry.update()
            /** Update loop info and play nice with the rest of the [Thread]s in the system  */
            loopCount++
        }
    }

    // emits a line of poetry to the telemetry log
    fun emitPoemLine() {
        telemetry.log().add(poem[poemLine])
        poemLine = (poemLine + 1) % poem.size
        poemElapsed.reset()
    }

    // Computes the current battery voltage
    val batteryVoltage: Double
        get() {
            var result = Double.POSITIVE_INFINITY
            for (sensor in hardwareMap.voltageSensor) {
                val voltage = sensor.voltage
                if (voltage > 0) {
                    result = Math.min(result, voltage)
                }
            }
            return result
        }

    companion object {
        val poem = arrayOf(
            "Mary had a little lamb,",
            "His fleece was white as snow,",
            "And everywhere that Mary went,",
            "The lamb was sure to go.",
            "",
            "He followed her to school one day,",
            "Which was against the rule,",
            "It made the children laugh and play",
            "To see a lamb at school.",
            "",
            "And so the teacher turned it out,",
            "But still it lingered near,",
            "And waited patiently about,",
            "Till Mary did appear.",
            "",
            "\"Why does the lamb love Mary so?\"",
            "The eager children cry.",
            "\"Why, Mary loves the lamb, you know,\"",
            "The teacher did reply.",
            "",
            ""
        )
    }
}
