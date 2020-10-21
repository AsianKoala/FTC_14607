//package org.firstinspires.ftc.teamcode.Teleop;
//
//import android.annotation.SuppressLint;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.PIDCoefficients;
//import com.qualcomm.robotcore.hardware.Servo;
//import org.firstinspires.ftc.teamcode.HelperClasses.ppProject.company.Range;
//import org.openftc.revextensions2.ExpansionHubEx;
//import org.openftc.revextensions2.ExpansionHubMotor;
//import org.openftc.revextensions2.RevBulkData;
//
//import java.util.ArrayList;
//
//
//import static org.firstinspires.ftc.teamcode.HelperClasses.GLOBALS.*;
//
//@TeleOp(name = "fruit fly teleop", group = "new")
//public class FruitFlyTeleOp extends OpMode {
//
//    private ExpansionHubMotor leftFront, leftRear, rightFront, rightRear;
//    private ExpansionHubMotor leftIntake, rightIntake;
//    private ExpansionHubMotor leftSlide, rightSlide;
//
//    private Servo leftFoundationGrabber, rightFoundationGrabber;
//    private Servo frontClaw, backClaw;
//    private Servo capstone;
//    private Servo park;
//    private Servo extension;
//
//    private ExpansionHubEx masterHub, slaveHub;
//
//
//
//
//    private ArrayList<ExpansionHubMotor> driveMotors = new ArrayList<ExpansionHubMotor>() {{
//        add(leftFront);
//        add(leftRear);
//        add(rightFront);
//        add(rightRear);
//    }};
//
//
//    private double newSlideLeft = 0;
//    private double newSlideRight = 0;
//
//
//    private double extensionPos = 0;
//
//
//    @Override
//    public void init() {
//
//        leftFront = hardwareMap.get(ExpansionHubMotor.class, "FL");
//        leftRear = hardwareMap.get(ExpansionHubMotor.class, "BL");
//        rightFront = hardwareMap.get(ExpansionHubMotor.class, "FR");
//        rightRear = hardwareMap.get(ExpansionHubMotor.class, "BR");
//        leftIntake = hardwareMap.get(ExpansionHubMotor.class, "leftIntake");
//        rightIntake = hardwareMap.get(ExpansionHubMotor.class, "rightIntake");
//        leftSlide = hardwareMap.get(ExpansionHubMotor.class, "leftSlide");
//        rightSlide = hardwareMap.get(ExpansionHubMotor.class, "rightSlide");
//
//
//        frontClaw = hardwareMap.get(Servo.class, "frontClaw");
//        backClaw = hardwareMap.get(Servo.class, "backClaw");
//        leftFoundationGrabber = hardwareMap.get(Servo.class, "leftFoundationGrabber");
//        rightFoundationGrabber = hardwareMap.get(Servo.class, "rightFoundationGrabber");
//        capstone = hardwareMap.get(Servo.class, "capstone");
//        park = hardwareMap.get(Servo.class, "park");
//        extension = hardwareMap.get(Servo.class, "extension");
//
//        masterHub = hardwareMap.get(ExpansionHubEx.class, "master");
//        slaveHub = hardwareMap.get(ExpansionHubEx.class, "slave");
//
//
//
//        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
//        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
//        rightIntake.setDirection(DcMotorSimple.Direction.REVERSE);
//        leftSlide.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        leftSlide.setTargetPosition(0);
//        rightSlide.setTargetPosition(0);
//        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        leftSlide.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDCoefficients(P,I,D));
//        rightSlide.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDCoefficients(P,I,D));
//
//
//        for(ExpansionHubMotor e : driveMotors) {
//            e.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            e.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        }
//
//
//        park.setPosition(parkPosition);
//        capstone.setPosition(capPosition);
//
//
//
//        telemetry.addLine("initialized");
//        telemetry.update();
//
//    }
//
//    @Override
//    public void init_loop() {
//
//    }
//
//
//    @Override
//    public void start() {
//
//    }
//
//    @Override
//    public void loop() {
//
//
//
//        // intake
//        double intakeMultiplier = 0.60; //todo: was 0.75
//        if(gamepad2.left_stick_button) {
//            intakeMultiplier = 1;
//        }
//        double leftIntakePower = gamepad2.left_stick_y - gamepad2.left_stick_x;
//        double rightIntakePower = gamepad2.left_stick_y + gamepad2.left_stick_x;
//        if(Math.abs(leftIntakePower) < 0.1 || Math.abs(rightIntakePower) < 0.1) {
//            leftIntake.setPower(0);
//            rightIntake.setPower(0);
//        }else {
//            leftIntake.setPower( intakeMultiplier * -leftIntakePower);
//            rightIntake.setPower( intakeMultiplier * -rightIntakePower);
//        }
//
//
//
//
//        // drive
//        double motorPower;
//        if(gamepad1.left_bumper) {
//            motorPower = 0.5;
//        }
//
//        else if(gamepad1.right_bumper) {
//            motorPower = 0.25;
//        }
//
//        else {
//            motorPower = 1;
//        }
//
//        double threshold = 0.157; // deadzone
//        if(Math.abs(gamepad1.left_stick_y) > threshold || Math.abs(gamepad1.left_stick_x) > threshold || Math.abs(gamepad1.right_stick_x) > threshold)
//        {
//            rightFront.setPower(motorPower * (((-gamepad1.left_stick_y) + (gamepad1.left_stick_x)) + -gamepad1.right_stick_x));
//            leftRear.setPower(motorPower * (((-gamepad1.left_stick_y) + (-gamepad1.left_stick_x)) + gamepad1.right_stick_x));
//            leftFront.setPower(motorPower * (((-gamepad1.left_stick_y) + (gamepad1.left_stick_x)) + gamepad1.right_stick_x));
//            rightRear.setPower(motorPower * (((-gamepad1.left_stick_y) + (-gamepad1.left_stick_x)) + -gamepad1.right_stick_x));
//        }
//
//        else
//        {
//            leftFront.setPower(0);
//            rightFront.setPower(0);
//            leftRear.setPower(0);
//            rightRear.setPower(0);
//        }
//
//
//
//
//        // vertical slide
//        double slideMultiplier = 80; //todo:100
//        if(gamepad2.right_stick_button) {
//            slideMultiplier = 20;
//        }
//
//
//        double increment = gamepad2.right_stick_y * slideMultiplier;
//        if(Math.abs(increment) > 5) {
//            if(leftSlide.getCurrentPosition()+increment < -15 && rightSlide.getCurrentPosition()+increment < -15)
//            {
//                newSlideLeft = leftSlide.getCurrentPosition() + increment;
//                newSlideRight = rightSlide.getCurrentPosition() + increment;
//
//            }
//            // stop slides from going down too much
//
//
//        }
//        if(gamepad2.x) {
//            newSlideLeft = -25;
//            newSlideRight = -25;
//        }
//
//        if(gamepad2.b) {
//            newSlideLeft = -14;
//            newSlideRight = -14;
//        }
//
//        if(newSlideLeft - leftSlide.getTargetPosition() < 0 || newSlideRight - rightSlide.getTargetPosition() < 0) {
//            leftSlide.setTargetPosition((int)(newSlideLeft));
//            rightSlide.setTargetPosition((int)(newSlideRight));
//            leftSlide.setPower(1);
//            rightSlide.setPower(1);
//        } else if(newSlideLeft - leftSlide.getTargetPosition() > 0 || newSlideRight - rightSlide.getTargetPosition() > 0) {
//            leftSlide.setTargetPosition((int)(newSlideLeft));
//            rightSlide.setTargetPosition((int)(newSlideRight));
//            leftSlide.setPower(0.3);
//            rightSlide.setPower(0.3);
//        }
//
//
//
//
//
//
//        // pepeJAM any jammers
//        // horizontal extension control
//        if(gamepad2.right_bumper) {
//            extensionPos += 0.05;
//        }
//
//        if(gamepad2.left_bumper) {
//            extensionPos -= 0.05;
//        }
//
//        extensionPos = Range.clip(extensionPos, 0, 1);
//        extension.setPosition(extensionPos);
//
//
//
//
//        //foundation mover control
//        if(gamepad1.a) {
//            grabFoundation();
//        }
//
//        if(gamepad1.b) {
//            ungrabFoundation();
//        }
//
//
//
//
//
//
//
//
//
//
//        telemetry.addLine(mecanumPowers());
//        telemetry.addData("left slide pos", leftSlide.getCurrentPosition());
//        telemetry.addData("right slide pos", rightSlide.getCurrentPosition());
//        telemetry.addData("left foundation pos", leftFoundationGrabber.getPosition());
//        telemetry.addData("right foundation pos", rightFoundationGrabber.getPosition());
//        telemetry.addData("capstone pos", capstone.getPosition());
//        telemetry.addData("front claw pos", frontClaw.getPosition());
//        telemetry.addData("back claw pos", backClaw.getPosition());
//        telemetry.addData("extension pos", extension.getPosition());
//        telemetry.addData("park pos", park.getPosition());
//        telemetry.addData("left intake power", leftIntake.getPower());
//        telemetry.addData("right intake power", rightIntake.getPower());
//
//    }
//
//
//
//    // feels strong man
//    @SuppressLint("DefaultLocale")
//    private String mecanumPowers() {
//        return String.format(
//                "\n" +
//                        "(%.1f)---(%.1f)\n" +
//                        "|   Front   |\n" +
//                        "|             |\n" +
//                        "|             |\n" +
//                        "(%.1f)---(%.1f)\n"
//                , leftFront.getPower(), rightFront.getPower(), leftRear.getPower(), rightRear.getPower());
//    }
//
//
//
//    private RevBulkData masterBulkData() { return masterHub.getBulkInputData(); }
//    private RevBulkData slaveBulkData() { return slaveHub.getBulkInputData(); }
//
//
//    private int getMotorBulkDataPosition(ExpansionHubEx hub, DcMotor motor) {
//        if(hub.equals(masterHub)) {
//            return masterBulkData().getMotorCurrentPosition(motor);
//        }
//
//        return slaveBulkData().getMotorCurrentPosition(motor);
//    }
//
//
//
//
//    private void grabFoundation() {
//        leftFoundationGrabber.setPosition(0.9);
//        rightFoundationGrabber.setPosition(0.1);
//    }
//
//    private void ungrabFoundation() {
//        leftFoundationGrabber.setPosition(0.1);
//        rightFoundationGrabber.setPosition(0.9);
//    }
//
//
//
//}
