package org.firstinspires.ftc.teamcode.treamcodde.teleop.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.ppProject.company.Robot;
import org.firstinspires.ftc.teamcode.treamcodde.HouseFly;


@TeleOp(name = "Firefly Teleop")
public class HouseFlyTeleop extends OpMode {

    private HouseFly robot;
    boolean isFieldOrientedDrive = true;
    double heading;

    @Override
    public void init() {
        robot = new HouseFly(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    // run until the end of the match (driver presses STOP)
    public void loop() {
        // intake motor control
        double leftIntakePower = gamepad2.left_stick_y - gamepad2.left_stick_x;
        double rightIntakePower = gamepad2.left_stick_y + gamepad2.left_stick_x;
        if(Math.abs(leftIntakePower) < 0.1 || Math.abs(rightIntakePower) < 0.1) {
            robot.leftIntake.setPower(0);
            robot.rightIntake.setPower(0);
        }else {
            robot.leftIntake.setPower(leftIntakePower);
            robot.rightIntake.setPower(rightIntakePower);
        }



        // vertical lift motor control
//        double leftLiftPower = -gamepad2.right_stick_y;
//        double rightLiftPower = -gamepad2.right_stick_y;
//        if(Math.abs(leftLiftPower) < 0.1 || Math.abs(rightLiftPower) < 0.1) {
//            robot.leftSlide.setPower(0);
//            robot.rightSlide.setPower(0);
//        }else {
//            robot.leftSlide.setPower(leftLiftPower);
//            robot.rightSlide.setPower(rightLiftPower);
//        }
        leftSlide.setPower(-gamepad2.right_stick_y);
        rightSlide.setPower(-gamepad2.right_stick_y);

        // flipper arm control
        if(gamepad2.right_trigger > 0.5) {
            robot.flip();
        }
        if(gamepad2.left_trigger > 0.5) {
            robot.flipReady();
        }

        // rotater arm control
        if(gamepad2.right_bumper) {
            robot.rotaterOut();
        }
        if(gamepad2.left_bumper) {
            robot.rotaterReady();
        }

        // gripper arm control
        if(gamepad2.a) {
            robot.grip();
        }
        if(gamepad2.y) {
            robot.gripReady();
        }



        // drivetrain control
        double motorPowerMultiplier;
        if(gamepad1.left_bumper) {
            motorPowerMultiplier = 0.5;
        }
        else if(gamepad1.right_bumper) {
            motorPowerMultiplier = 0.25;
        }
        else {
            motorPowerMultiplier = 1;
        }

        if(gamepad1.a) {
            isFieldOrientedDrive = true;
        }
        if(gamepad1.y) {
            isFieldOrientedDrive = false;
        }



        double threshold = 0.157; // deadzone
        if(Math.abs(gamepad1.left_stick_y) > threshold || Math.abs(gamepad1.left_stick_x) > threshold || Math.abs(gamepad1.right_stick_x) > threshold)
        {
            if(isFieldOrientedDrive) {
                heading = robot.getRawExternalHeading();

                double magnitude = Math.sqrt(Math.pow(gamepad1.left_stick_y, 2) + Math.pow(gamepad1.left_stick_x, 2));
                double driveHeading = Math.atan2(gamepad1.left_stick_y/gamepad1.left_stick_x);
                driveHeading *= 180/Math.PI;
//                if(gamepad1.left_stick_y < 0 && gamepad1.left_stick_x < 0) {
//                    driveHeading += 180.0;
//                }
//                if(gamepad1.left_stick_y < 0 && gamepad1.left_stick_x > 0) {
//                    // then arctan returns correct value
//                }
//                if(gamepad1.left_stick_y > 0 && gamepad1.left_stick_x < 0) {
//                    driveHeading += 180.0;
//                }
                double theta = driveHeading-heading;
                double robotXcomponent = Math.cos(theta) * magnitude;
                double robotYcomponent = Math.sin(theta) * magnitude;
                double strafeMultipler = 1/Math.sqrt(2); // idk if this is correct

                double virtualLeftStickX = robotXcomponent * (1/strafeMultipler);
                double virtualLeftStickY = robotYcomponent;

                rightFront.setPower(motorPowerMultiplier * (((-virtualLeftStickY) + (virtualLeftStickX)) + -gamepad1.right_stick_x));
                leftRear.setPower(motorPowerMultiplier * (((-virtualLeftStickY) + (-virtualLeftStickX)) + gamepad1.right_stick_x));
                leftFront.setPower(motorPowerMultiplier * (((-virtualLeftStickY) + (virtualLeftStickX)) + gamepad1.right_stick_x));
                rightRear.setPower(motorPowerMultiplier * (((-virtualLeftStickY) + (-virtualLeftStickX)) + -gamepad1.right_stick_x));
            }else {
                rightFront.setPower(motorPowerMultiplier * (((-gamepad1.left_stick_y) + (gamepad1.left_stick_x)) + -gamepad1.right_stick_x));
                leftRear.setPower(motorPowerMultiplier * (((-gamepad1.left_stick_y) + (-gamepad1.left_stick_x)) + gamepad1.right_stick_x));
                leftFront.setPower(motorPowerMultiplier * (((-gamepad1.left_stick_y) + (gamepad1.left_stick_x)) + gamepad1.right_stick_x));
                rightRear.setPower(motorPowerMultiplier * (((-gamepad1.left_stick_y) + (-gamepad1.left_stick_x)) + -gamepad1.right_stick_x));
            }
        }
        else
        {
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftRear.setPower(0);
            rightRear.setPower(0);
        }



    }
}

