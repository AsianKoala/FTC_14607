package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.HouseFly_Hardware;

@TeleOp(name = "not that scuffed :P jk sucks", group = "")
public class simpleprettyscuffedteleop extends OpMode {
    HouseFly_Hardware robot = new HouseFly_Hardware();
    public double threshold = 0.157;

    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void init_loop() {
        telemetry.addData("",". . .");
        telemetry.update();
        telemetry.addData("","* * *");
        telemetry.update();
    }

    @Override
    public void start() {
        // runs once when start
    }

    @Override
    public void loop() {
        // loops until stop
        if(gamepad1.start && gamepad2.back) {
            robot.resetEncoders();
        }

        if(Math.abs(gamepad1.left_stick_y) > threshold || Math.abs(gamepad1.left_stick_x) > threshold)
        {
            robot.frontRight.setPower(((-gamepad1.left_stick_y) + (-gamepad1.left_stick_x))); // rf
            robot.backLeft.setPower(((-gamepad1.left_stick_y) + (-gamepad1.left_stick_x))); // lr
            robot.frontLeft.setPower(((-gamepad1.left_stick_y) - (-gamepad1.left_stick_x))); // lf
            robot.backRight.setPower(((-gamepad1.left_stick_y) - (-gamepad1.left_stick_x))); // rr
        }

        else
        {
            robot.frontLeft.setPower(0); //lf
            robot.frontRight.setPower(0); //rf
            robot.backLeft.setPower(0); //lr
            robot.backRight.setPower(0); //rr
        }

        if(Math.abs(gamepad1.right_stick_x) > threshold)
        {
            robot.frontRight.setPower((-gamepad1.right_stick_x)); //fr
            robot.frontLeft.setPower((gamepad1.right_stick_x)); //fl
            robot.backLeft.setPower((gamepad1.right_stick_x)); //bl
            robot.backRight.setPower((-gamepad1.right_stick_x)); //br
        }



        // telemetry

        telemetry.addData("fl power", robot.frontLeft.getPower());
        telemetry.addData("fr power", robot.frontRight.getPower());
        telemetry.addData("bl power", robot.backLeft.getPower());
        telemetry.addData("br power", robot.backRight.getPower());

        telemetry.addData("fl power", robot.frontLeft.getCurrentPosition());
        telemetry.addData("fr power", robot.frontRight.getCurrentPosition());
        telemetry.addData("bl power", robot.backLeft.getCurrentPosition());
        telemetry.addData("br power", robot.backRight.getCurrentPosition());

        telemetry.update();

    }

    @Override
    public void stop() {

    }
}
