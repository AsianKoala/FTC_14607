package org.firstinspires.ftc.teamcode.teleop.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "basiceleopdrve")
public class HouseFlyTeleop extends OpMode {

    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftRear;
    private DcMotor rightRear;
    private DcMotor leftIntake;
    private DcMotor rightIntake;
    private DcMotor leftSlide;
    private DcMotor rightSlide;
    private Servo flipper, gripper, outtake, leftSlam, rightSlam;



    private double movement_x;
    private double movement_y;
    private double movement_turn;

    // TODO: EDIT THIS SUTFF
    private double flipperFlipPosition = 1;
    private double flipperReadyPosition = 0;
    private double gripperGripPosition = 1;
    private double gripperReadyPosition = 0.5;
    private double outtakeOutPosition = 0;
    private double outtakeReadyPosition = 1;




    @Override
    public void init() {

        leftFront = hardwareMap.get(DcMotor.class, "FL");
        leftRear = hardwareMap.get(DcMotor.class, "BL");
        rightRear = hardwareMap.get(DcMotor.class, "FR");
        rightFront = hardwareMap.get(DcMotor.class, "BR");
        leftIntake = hardwareMap.get(DcMotor.class, "leftIntake");
        rightIntake = hardwareMap.get(DcMotor.class, "rightIntake");
        leftSlide = hardwareMap.get(DcMotor.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotor.class, "rightSlide");

        gripper = hardwareMap.get(Servo.class, "gripper");
        flipper = hardwareMap.get(Servo.class, "flipper");
        outtake = hardwareMap.get(Servo.class, "outtake");
        leftSlam = hardwareMap.get(Servo.class, "leftSlam");
        rightSlam = hardwareMap.get(Servo.class, "rightSlam");



        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        rightIntake.setDirection(DcMotor.Direction.REVERSE);
        leftSlide.setDirection(DcMotor.Direction.REVERSE);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    // run until the end of the match (driver presses STOP)
    public void loop() {
        //Drive motor control

        leftIntake.setPower(gamepad2.left_stick_y);
        rightIntake.setPower(-gamepad2.left_stick_y);
        leftSlide.setPower(-gamepad2.right_stick_y);
        rightSlide.setPower(-gamepad2.right_stick_y);




        double threshold = 0.157;
        if(Math.abs(gamepad1.left_stick_y) > threshold || Math.abs(gamepad1.left_stick_x) > threshold || Math.abs(gamepad1.right_stick_x) > threshold)
        {
            rightFront.setPower(((-gamepad1.left_stick_y) + (gamepad1.left_stick_x)) + -gamepad1.right_stick_x);
            leftRear.setPower(((-gamepad1.left_stick_y) + (-gamepad1.left_stick_x)) + gamepad1.right_stick_x);
            leftFront.setPower(((-gamepad1.left_stick_y) + (gamepad1.left_stick_x)) + gamepad1.right_stick_x);
            rightRear.setPower(((-gamepad1.left_stick_y) + (-gamepad1.left_stick_x)) + -gamepad1.right_stick_x);
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

