package org.firstinspires.ftc.teamcode.treamcodde.teleop.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.ppProject.company.Robot;
import org.firstinspires.ftc.teamcode.treamcodde.HouseFly;


@TeleOp(name = "basiceleopdrve")
public class HouseFlyTeleop extends OpMode {

    private HouseFly robot;

    /**
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     * charlie theres a ton of errors so can you fix them if I haven't fixedf them already
     * srry its like 1.5 am
     * 
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     *
     */
    @Override
    public void init() {


        robot = new HouseFly(hardwareMap);


        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    // run until the end of the match (driver presses STOP)
    public void loop() {
        //Drive motor control


        robot.leftIntake.setPower(gamepad1.left_stick_y - gamepad1.left_stick_x );
        robot.rightIntake.setPower(-gamepad1.left_stick_y - -gamepad1.left_stick_x);

        if(gamepad2.left_bumper) {
            rightIntake.setPower(-1);
            leftIntake.setPower(-1);
        }

        else if(gamepad2.right_bumper) {
            rightIntake.setPower(1);
            leftIntake.setPower(-.1);
        }

        else {
            rightIntake.setPower(0);
            leftIntake.setPower(0);
        }




        leftSlide.setPower(-gamepad2.right_stick_y);
        rightSlide.setPower(-gamepad2.right_stick_y);


        double motorPower;
        if(gamepad1.left_bumper) {
            motorPower = 0.5;
        }

        else if(gamepad1.right_bumper) {
            motorPower = 0.25;
        }

        else {
            motorPower = 1;
        }



        /*
        drive motor powers
         */
        double threshold = 0.157; // deadzone
        if(Math.abs(gamepad1.left_stick_y) > threshold || Math.abs(gamepad1.left_stick_x) > threshold || Math.abs(gamepad1.right_stick_x) > threshold)
        {
            rightFront.setPower(motorPower * (((-gamepad1.left_stick_y) + (gamepad1.left_stick_x)) + -gamepad1.right_stick_x));
            leftRear.setPower(motorPower * (((-gamepad1.left_stick_y) + (-gamepad1.left_stick_x)) + gamepad1.right_stick_x));
            leftFront.setPower(motorPower * (((-gamepad1.left_stick_y) + (gamepad1.left_stick_x)) + gamepad1.right_stick_x));
            rightRear.setPower(motorPower * (((-gamepad1.left_stick_y) + (-gamepad1.left_stick_x)) + -gamepad1.right_stick_x));
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

