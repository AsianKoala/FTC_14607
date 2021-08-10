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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.ElapsedTime
import com.qualcomm.robotcore.util.Range

/**
 * This file illustrates the concept of driving a path based on Gyro heading and encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 * otherwise you would use: PushbotAutoDriveByTime;
 *
 *
 * This code ALSO requires that you have a Modern Robotics I2C gyro with the name "gyro"
 * otherwise you would use: PushbotAutoDriveByEncoder;
 *
 *
 * This code requires that the drive Motors have been configured such that a positive
 * power command moves them forward, and causes the encoders to count UP.
 *
 *
 * This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 *
 * In order to calibrate the Gyro correctly, the robot must remain stationary during calibration.
 * This is performed when the INIT button is pressed on the Driver Station.
 * This code assumes that the robot is stationary when the INIT button is pressed.
 * If this is not the case, then the INIT should be performed again.
 *
 *
 * Note: in this example, all angles are referenced to the initial coordinate frame set during the
 * the Gyro Calibration process, or whenever the program issues a resetZAxisIntegrator() call on the Gyro.
 *
 *
 * The angle of movement/rotation is assumed to be a standardized rotation around the robot Z axis,
 * which means that a Positive rotation is Counter Clock Wise, looking down on the field.
 * This is consistent with the FTC field coordinate conventions set out in the document:
 * ftc_app\doc\tutorial\FTC_FieldCoordinateSystemDefinition.pdf
 *
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@Autonomous(name = "Pushbot: Auto Drive By Gyro", group = "Pushbot")
@Disabled
class PushbotAutoDriveByGyro_Linear constructor() : LinearOpMode() {
    /* Declare OpMode members. */
    var robot: HardwarePushbot = HardwarePushbot() // Use a Pushbot's hardware
    var gyro: ModernRoboticsI2cGyro? = null // Additional Gyro device
    public override fun runOpMode() {

        /*
         * Initialize the standard drive system variables.
         * The init() method of the hardware class does most of the work here
         */
        robot.init(hardwareMap)
        gyro = hardwareMap.gyroSensor.get("gyro") as ModernRoboticsI2cGyro?

        // Ensure the robot it stationary, then reset the encoders and calibrate the gyro.
        robot.leftDrive!!.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
        robot.rightDrive!!.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)

        // Send telemetry message to alert driver that we are calibrating;
        telemetry.addData(">", "Calibrating Gyro") //
        telemetry.update()
        gyro!!.calibrate()

        // make sure the gyro is calibrated before continuing
        while (!isStopRequested() && gyro!!.isCalibrating()) {
            sleep(50)
            idle()
        }
        telemetry.addData(">", "Robot Ready.") //
        telemetry.update()
        robot.leftDrive!!.setMode(DcMotor.RunMode.RUN_USING_ENCODER)
        robot.rightDrive!!.setMode(DcMotor.RunMode.RUN_USING_ENCODER)

        // Wait for the game to start (Display Gyro value), and reset gyro before we move..
        while (!isStarted()) {
            telemetry.addData(">", "Robot Heading = %d", gyro!!.getIntegratedZValue())
            telemetry.update()
        }
        gyro!!.resetZAxisIntegrator()

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        // Put a hold after each turn
        gyroDrive(DRIVE_SPEED, 48.0, 0.0) // Drive FWD 48 inches
        gyroTurn(TURN_SPEED, -45.0) // Turn  CCW to -45 Degrees
        gyroHold(TURN_SPEED, -45.0, 0.5) // Hold -45 Deg heading for a 1/2 second
        gyroDrive(DRIVE_SPEED, 12.0, -45.0) // Drive FWD 12 inches at 45 degrees
        gyroTurn(TURN_SPEED, 45.0) // Turn  CW  to  45 Degrees
        gyroHold(TURN_SPEED, 45.0, 0.5) // Hold  45 Deg heading for a 1/2 second
        gyroTurn(TURN_SPEED, 0.0) // Turn  CW  to   0 Degrees
        gyroHold(TURN_SPEED, 0.0, 1.0) // Hold  0 Deg heading for a 1 second
        gyroDrive(DRIVE_SPEED, -48.0, 0.0) // Drive REV 48 inches
        telemetry.addData("Path", "Complete")
        telemetry.update()
    }

    /**
     * Method to drive on a fixed compass bearing (angle), based on encoder counts.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the desired position
     * 2) Driver stops the opmode running.
     *
     * @param speed    Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param distance Distance (in inches) to move from current position.  Negative distance means move backwards.
     * @param angle    Absolute Angle (in Degrees) relative to last gyro reset.
     * 0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     * If a relative angle is required, add/subtract from current heading.
     */
    fun gyroDrive(
        speed: Double,
        distance: Double,
        angle: Double
    ) {
        var speed: Double = speed
        val newLeftTarget: Int
        val newRightTarget: Int
        val moveCounts: Int
        var max: Double
        var error: Double
        var steer: Double
        var leftSpeed: Double
        var rightSpeed: Double

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (distance * COUNTS_PER_INCH).toInt()
            newLeftTarget = robot.leftDrive!!.getCurrentPosition() + moveCounts
            newRightTarget = robot.rightDrive!!.getCurrentPosition() + moveCounts

            // Set Target and Turn On RUN_TO_POSITION
            robot.leftDrive!!.setTargetPosition(newLeftTarget)
            robot.rightDrive!!.setTargetPosition(newRightTarget)
            robot.leftDrive!!.setMode(DcMotor.RunMode.RUN_TO_POSITION)
            robot.rightDrive!!.setMode(DcMotor.RunMode.RUN_TO_POSITION)

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0)
            robot.leftDrive!!.setPower(speed)
            robot.rightDrive!!.setPower(speed)

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                (robot.leftDrive!!.isBusy() && robot.rightDrive!!.isBusy())
            ) {

                // adjust relative speed based on heading error.
                error = getError(angle)
                steer = getSteer(error, P_DRIVE_COEFF)

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0) steer *= -1.0
                leftSpeed = speed - steer
                rightSpeed = speed + steer

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed))
                if (max > 1.0) {
                    leftSpeed /= max
                    rightSpeed /= max
                }
                robot.leftDrive!!.setPower(leftSpeed)
                robot.rightDrive!!.setPower(rightSpeed)

                // Display drive status for the driver.
                telemetry.addData("Err/St", "%5.1f/%5.1f", error, steer)
                telemetry.addData("Target", "%7d:%7d", newLeftTarget, newRightTarget)
                telemetry.addData(
                    "Actual", "%7d:%7d", robot.leftDrive!!.getCurrentPosition(),
                    robot.rightDrive!!.getCurrentPosition()
                )
                telemetry.addData("Speed", "%5.2f:%5.2f", leftSpeed, rightSpeed)
                telemetry.update()
            }

            // Stop all motion;
            robot.leftDrive!!.setPower(0.0)
            robot.rightDrive!!.setPower(0.0)

            // Turn off RUN_TO_POSITION
            robot.leftDrive!!.setMode(DcMotor.RunMode.RUN_USING_ENCODER)
            robot.rightDrive!!.setMode(DcMotor.RunMode.RUN_USING_ENCODER)
        }
    }

    /**
     * Method to spin on central axis to point in a new direction.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the heading (angle)
     * 2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle Absolute Angle (in Degrees) relative to last gyro reset.
     * 0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     * If a relative angle is required, add/subtract from current heading.
     */
    fun gyroTurn(speed: Double, angle: Double) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update()
        }
    }

    /**
     * Method to obtain & hold a heading for a finite amount of time
     * Move will stop once the requested time has elapsed
     *
     * @param speed    Desired speed of turn.
     * @param angle    Absolute Angle (in Degrees) relative to last gyro reset.
     * 0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     * If a relative angle is required, add/subtract from current heading.
     * @param holdTime Length of time (in seconds) to hold the specified heading.
     */
    fun gyroHold(speed: Double, angle: Double, holdTime: Double) {
        val holdTimer: ElapsedTime = ElapsedTime()

        // keep looping while we have time remaining.
        holdTimer.reset()
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF)
            telemetry.update()
        }

        // Stop all motion;
        robot.leftDrive!!.setPower(0.0)
        robot.rightDrive!!.setPower(0.0)
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed  Desired speed of turn.
     * @param angle  Absolute Angle (in Degrees) relative to last gyro reset.
     * 0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     * If a relative angle is required, add/subtract from current heading.
     * @param PCoeff Proportional Gain coefficient
     * @return
     */
    fun onHeading(speed: Double, angle: Double, PCoeff: Double): Boolean {
        val error: Double
        val steer: Double
        var onTarget: Boolean = false
        val leftSpeed: Double
        val rightSpeed: Double

        // determine turn power based on +/- error
        error = getError(angle)
        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0
            leftSpeed = 0.0
            rightSpeed = 0.0
            onTarget = true
        } else {
            steer = getSteer(error, PCoeff)
            rightSpeed = speed * steer
            leftSpeed = -rightSpeed
        }

        // Send desired speeds to motors.
        robot.leftDrive!!.setPower(leftSpeed)
        robot.rightDrive!!.setPower(rightSpeed)

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle)
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer)
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed)
        return onTarget
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     *
     * @param targetAngle Desired angle (relative to global reference established at last Gyro Reset).
     * @return error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     * +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    fun getError(targetAngle: Double): Double {
        var robotError: Double

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - gyro!!.getIntegratedZValue()
        while (robotError > 180) robotError -= 360.0
        while (robotError <= -180) robotError += 360.0
        return robotError
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     *
     * @param error  Error angle in robot relative degrees
     * @param PCoeff Proportional Gain Coefficient
     * @return
     */
    fun getSteer(error: Double, PCoeff: Double): Double {
        return Range.clip(error * PCoeff, -1.0, 1.0)
    }

    companion object {
        val COUNTS_PER_MOTOR_REV: Double = 1440.0 // eg: TETRIX Motor Encoder
        val DRIVE_GEAR_REDUCTION: Double = 2.0 // This is < 1.0 if geared UP
        val WHEEL_DIAMETER_INCHES: Double = 4.0 // For figuring circumference
        val COUNTS_PER_INCH: Double = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415)

        // These constants define the desired driving/control characteristics
        // The can/should be tweaked to suite the specific robot drive train.
        val DRIVE_SPEED: Double = 0.7 // Nominal speed for better accuracy.
        val TURN_SPEED: Double = 0.5 // Nominal half speed for better accuracy.
        val HEADING_THRESHOLD: Double = 1.0 // As tight as we can make it with an integer gyro
        val P_TURN_COEFF: Double = 0.1 // Larger is more responsive, but also less stable
        val P_DRIVE_COEFF: Double = 0.15 // Larger is more responsive, but also less stable
    }
}
