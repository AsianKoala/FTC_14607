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

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.IntegratingGyroscope
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference

/*
 * This is an example LinearOpMode that shows how to use the Modern Robotics Gyro.
 *
 * The op mode assumes that the gyro sensor is attached to a Device Interface Module
 * I2C channel and is configured with a name of "gyro".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@TeleOp(name = "Sensor: MR Gyro", group = "Sensor")
@Disabled
class SensorMRGyro : LinearOpMode() {
    /**
     * In this sample, for illustration purposes we use two interfaces on the one gyro object.
     * That's likely atypical: you'll probably use one or the other in any given situation,
     * depending on what you're trying to do. [IntegratingGyroscope] (and it's base interface,
     * [Gyroscope]) are common interfaces supported by possibly several different gyro
     * implementations. [ModernRoboticsI2cGyro], by contrast, provides functionality that
     * is unique to the Modern Robotics gyro sensor.
     */
    lateinit var gyro: IntegratingGyroscope
    lateinit var modernRoboticsI2cGyro: ModernRoboticsI2cGyro

    // A timer helps provide feedback while calibration is taking place
    var timer = ElapsedTime()
    override fun runOpMode() {
        var lastResetState = false
        var curResetState = false

        // Get a reference to a Modern Robotics gyro object. We use several interfaces
        // on this object to illustrate which interfaces support which functionality.
        modernRoboticsI2cGyro = hardwareMap.get(ModernRoboticsI2cGyro::class.java, "gyro")
        gyro = modernRoboticsI2cGyro
        // If you're only interested int the IntegratingGyroscope interface, the following will suffice.
        // gyro = hardwareMap.get(IntegratingGyroscope.class, "gyro");
        // A similar approach will work for the Gyroscope interface, if that's all you need.

        // Start calibrating the gyro. This takes a few seconds and is worth performing
        // during the initialization phase at the start of each opMode.
        telemetry.log().add("Gyro Calibrating. Do Not Move!")
        modernRoboticsI2cGyro.calibrate()

        // Wait until the gyro calibration is complete
        timer.reset()
        while (!isStopRequested && modernRoboticsI2cGyro.isCalibrating()) {
            telemetry.addData(
                "calibrating",
                "%s",
                if (Math.round(timer.seconds()) % 2 == 0L) "|.." else "..|"
            )
            telemetry.update()
            sleep(50)
        }
        telemetry.log().clear()
        telemetry.log().add("Gyro Calibrated. Press Start.")
        telemetry.clear()
        telemetry.update()

        // Wait for the start button to be pressed
        waitForStart()
        telemetry.log().clear()
        telemetry.log().add("Press A & B to reset heading")

        // Loop until we're asked to stop
        while (opModeIsActive()) {

            // If the A and B buttons are pressed just now, reset Z heading.
            curResetState = gamepad1.a && gamepad1.b
            if (curResetState && !lastResetState) {
                modernRoboticsI2cGyro.resetZAxisIntegrator()
            }
            lastResetState = curResetState

            // The raw() methods report the angular rate of change about each of the
            // three axes directly as reported by the underlying sensor IC.
            val rawX = modernRoboticsI2cGyro.rawX()
            val rawY = modernRoboticsI2cGyro.rawY()
            val rawZ = modernRoboticsI2cGyro.rawZ()
            val heading = modernRoboticsI2cGyro.getHeading()
            val integratedZ = modernRoboticsI2cGyro.getIntegratedZValue()

            // Read dimensionalized data from the gyro. This gyro can report angular velocities
            // about all three axes. Additionally, it internally integrates the Z axis to
            // be able to report an absolute angular Z orientation.
            val rates = gyro!!.getAngularVelocity(AngleUnit.DEGREES)
            val zAngle = gyro!!.getAngularOrientation(
                AxesReference.INTRINSIC,
                AxesOrder.ZYX,
                AngleUnit.DEGREES
            ).firstAngle

            // Read administrative information from the gyro
            val zAxisOffset = modernRoboticsI2cGyro.getZAxisOffset()
            val zAxisScalingCoefficient = modernRoboticsI2cGyro.getZAxisScalingCoefficient()
            telemetry.addLine()
                .addData("dx", formatRate(rates.xRotationRate))
                .addData("dy", formatRate(rates.yRotationRate))
                .addData("dz", "%s deg/s", formatRate(rates.zRotationRate))
            telemetry.addData("angle", "%s deg", formatFloat(zAngle))
            telemetry.addData("heading", "%3d deg", heading)
            telemetry.addData("integrated Z", "%3d", integratedZ)
            telemetry.addLine()
                .addData("rawX", formatRaw(rawX))
                .addData("rawY", formatRaw(rawY))
                .addData("rawZ", formatRaw(rawZ))
            telemetry.addLine().addData("z offset", zAxisOffset)
                .addData("z coeff", zAxisScalingCoefficient)
            telemetry.update()
        }
    }

    fun formatRaw(rawValue: Int): String {
        return String.format("%d", rawValue)
    }

    fun formatRate(rate: Float): String {
        return String.format("%.3f", rate)
    }

    fun formatFloat(rate: Float): String {
        return String.format("%.3f", rate)
    }
}
