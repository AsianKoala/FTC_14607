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

import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import org.firstinspires.ftc.robotcore.external.navigation.Orientation
import org.firstinspires.ftc.robotcore.external.navigation.Position
import org.firstinspires.ftc.robotcore.external.navigation.Velocity
import java.util.Locale

/**
 * [SensorBNO055IMU] gives a short demo on how to use the BNO055 Inertial Motion Unit (IMU) from AdaFruit.
 *
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 *
 * @see [Adafruit IMU](http://www.adafruit.com/products/2472)
 */
@TeleOp(name = "Sensor: BNO055 IMU", group = "Sensor")
@Disabled // Comment this out to add to the opmode list
class SensorBNO055IMU : LinearOpMode() {
    // ----------------------------------------------------------------------------------------------
    // State
    // ----------------------------------------------------------------------------------------------
    // The IMU sensor object
    lateinit var imu: BNO055IMU

    // State used for updating telemetry
    lateinit var angles: Orientation
    lateinit var gravity: Acceleration

    // ----------------------------------------------------------------------------------------------
    // Main logic
    // ----------------------------------------------------------------------------------------------
    override fun runOpMode() {

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        val parameters = BNO055IMU.Parameters()
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC
        parameters.calibrationDataFile =
            "BNO055IMUCalibration.json" // see the calibration sample opmode
        parameters.loggingEnabled = true
        parameters.loggingTag = "IMU"
        parameters.accelerationIntegrationAlgorithm = JustLoggingAccelerationIntegrator()

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU::class.java, "imu")
        imu.initialize(parameters)

        // Set up our telemetry dashboard
        composeTelemetry()

        // Wait until we're told to go
        waitForStart()

        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(Position(), Velocity(), 1000)

        // Loop and update the dashboard
        while (opModeIsActive()) {
            telemetry.update()
        }
    }

    // ----------------------------------------------------------------------------------------------
    // Telemetry Configuration
    // ----------------------------------------------------------------------------------------------
    fun composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction { // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles = imu!!.getAngularOrientation(
                AxesReference.INTRINSIC,
                AxesOrder.ZYX,
                AngleUnit.DEGREES
            )
            gravity = imu!!.gravity
        }
        telemetry.addLine()
            .addData("status") { imu!!.systemStatus.toShortString() }
            .addData("calib") { imu!!.calibrationStatus.toString() }
        telemetry.addLine()
            .addData("heading") { formatAngle(angles!!.angleUnit, angles!!.firstAngle.toDouble()) }
            .addData("roll") { formatAngle(angles!!.angleUnit, angles!!.secondAngle.toDouble()) }
            .addData("pitch") { formatAngle(angles!!.angleUnit, angles!!.thirdAngle.toDouble()) }
        telemetry.addLine()
            .addData("grvty") { gravity.toString() }
            .addData("mag") {
                String.format(
                    Locale.getDefault(), "%.3f",
                    Math.sqrt(gravity!!.xAccel * gravity!!.xAccel + gravity!!.yAccel * gravity!!.yAccel + gravity!!.zAccel * gravity!!.zAccel)
                )
            }
    }

    // ----------------------------------------------------------------------------------------------
    // Formatting
    // ----------------------------------------------------------------------------------------------
    fun formatAngle(angleUnit: AngleUnit?, angle: Double): String {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle))
    }

    fun formatDegrees(degrees: Double): String {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees))
    }
}
