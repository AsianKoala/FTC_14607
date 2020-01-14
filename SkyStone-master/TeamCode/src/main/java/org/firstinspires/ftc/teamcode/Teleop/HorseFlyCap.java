package org.firstinspires.ftc.teamcode.Teleop;
//BASED OFF OF AUTOMATED TELEOP DOES NOT ACCOUNT FOR NEW RESTRUCTURING

import android.annotation.SuppressLint;
import android.os.SystemClock;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import net.frogbots.ftcopmodetunercommon.opmode.TunableOpMode;

import org.firstinspires.ftc.teamcode.Auto.DriveConstants;
import org.firstinspires.ftc.teamcode.HelperClasses.GLOBALS;
import org.openftc.revextensions2.ExpansionHubMotor;

import static org.firstinspires.ftc.teamcode.HelperClasses.GLOBALS.*;

import java.util.ArrayList;



@TeleOp(name = "MAIN teleop control")
public class HorseFlyCap extends TunableOpMode {

    /**
     * LIST OF TODOS
     * TODO: add integration with hardware class
     * TODO: add field centric drive that charlie coded
     * TODO: add more state machine stuff so its easier for the driver to use robot
     * TODO: add more stuff that makes it easier for driver to drive
     */        double thime =0;

    private ExpansionHubMotor leftFront;
    private ExpansionHubMotor rightFront;
    private ExpansionHubMotor leftRear;
    private ExpansionHubMotor rightRear;
    private ExpansionHubMotor leftIntake;
    private ExpansionHubMotor rightIntake;
    private ExpansionHubMotor leftSlide;
    private ExpansionHubMotor rightSlide;
    private Servo flipper, gripper, rotater, leftSlam, rightSlam, capstone, park;

    private ArrayList<ExpansionHubMotor> driveMotors = new ArrayList<>();






    public final static long toMidTime = 450;
    public final static long liftTime = 200;//is for rotater now
    public final static long toBackTime = 750;// is for flipper now

    public final static long toLiftTimeTo = 400;
    public final static long toBackTimeTo = 700;

    public final static int liftIncrement = -200;
    public final static int liftIncrementer = -500;

    private double oldSlideLeft = 0;
    private double oldSlideRight = 0;
    private double newSlideLeft = 0;
    private double newSlideRight = 0;

    public static long time = 0;
    public static int count = 0;

    public static long chime = 0;
    public static int counter = 0;

    private double capPosition = 0.75;
    private double parkPosition = 0.83;


    @Override
    public void init() {

        leftFront = hardwareMap.get(ExpansionHubMotor.class, "FL");
        leftRear = hardwareMap.get(ExpansionHubMotor.class, "BL");
        rightRear = hardwareMap.get(ExpansionHubMotor.class, "FR");
        rightFront = hardwareMap.get(ExpansionHubMotor.class, "BR");
        leftIntake = hardwareMap.get(ExpansionHubMotor.class, "leftIntake");
        rightIntake = hardwareMap.get(ExpansionHubMotor.class, "rightIntake");
        leftSlide = hardwareMap.get(ExpansionHubMotor.class, "leftSlide");
        rightSlide = hardwareMap.get(ExpansionHubMotor.class, "rightSlide");

        gripper = hardwareMap.get(Servo.class, "gripper");
        flipper = hardwareMap.get(Servo.class, "flipper");
        rotater = hardwareMap.get(Servo.class, "rotater");
        leftSlam = hardwareMap.get(Servo.class, "leftSlam");
        rightSlam = hardwareMap.get(Servo.class, "rightSlam");
        capstone = hardwareMap.get(Servo.class, "capstone");

        park = hardwareMap.get(Servo.class, "park");


        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        rightIntake.setDirection(DcMotor.Direction.REVERSE);
        leftSlide.setDirection(DcMotor.Direction.REVERSE);


        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        leftSlide.setTargetPosition(0);
        rightSlide.setTargetPosition(0);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftSlide.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDCoefficients(P,I,D));
        rightSlide.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDCoefficients(P,I,D));




        driveMotors.add(leftRear);
        driveMotors.add(leftFront);
        driveMotors.add(rightFront);
        driveMotors.add(rightRear);
        /*
         * HOME THE FLIP AND GRIP SERVO
         */
//        capstone.setPosition(capBetween);
        flipReady();
        rotaterReady();
        gripReady();

        park.setPosition(parkPosition);
        capstone.setPosition(capPosition);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }


    public void loop() {


        // tunable opmode vars





        /**
         *
         * INTAKE CONTROL
         *
         */
        double intakeMultiplier = 0.60; //todo: was 0.75
        if(gamepad2.left_stick_button) {
            intakeMultiplier = 1;
        }
        double leftIntakePower = gamepad2.left_stick_y - gamepad2.left_stick_x;
        double rightIntakePower = gamepad2.left_stick_y + gamepad2.left_stick_x;
        if(Math.abs(leftIntakePower) < 0.1 || Math.abs(rightIntakePower) < 0.1) {
            leftIntake.setPower(0);
            rightIntake.setPower(0);
        }else {
            leftIntake.setPower( intakeMultiplier * -leftIntakePower);
            rightIntake.setPower( intakeMultiplier * -rightIntakePower);
        }




        /**



         DRIVE MOTORS CONTROL



         */

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

//        if(gamepad1.left_stick_button) {
//            capstone.setPosition(capBetween);
//        }else if(gamepad1.right_trigger > 0.5) {
//            capstone.setPosition(capUp);
//        }else{
//            capstone.setPosition(capDown);
//        }
        if(gamepad1.left_trigger > 0.5) {
            capPosition+=0.01;
            capstone.setPosition(capPosition);
        }

        if(gamepad1.right_trigger > 0.5) {
            capPosition-=0.01;
            capstone.setPosition(capPosition);
        }

        if(gamepad1.dpad_down) {
            parkPosition+=0.025;
            park.setPosition(parkPosition);
        }

        if(gamepad1.dpad_up) {
            parkPosition-=0.025;
            park.setPosition(parkPosition);
        }








        // flipper arm control

        if(gamepad2.dpad_down) {
            double gime = System.currentTimeMillis();
//            if(System.currentTimeMillis()-gime <500) {
////                capstone.setPosition(capUp);
//            }
            flipper.setPosition(flipperBetween);
        }

        if(gamepad2.dpad_up) {
//            capstone.setPosition(capUp);
        }

        if(gamepad2.right_trigger > 0.5) {
            flipper.setPosition(flipperHome);
        }
        if(gamepad2.left_trigger > 0.5) {
            flipper.setPosition(flipperOut);
        }


        // rotater arm control
        if(gamepad2.left_bumper) {
            rotaterOut();
        }
        if(gamepad2.right_bumper) {
            rotaterReady();
        }


        // gripper arm control
        if(gamepad2.a) {
            gripReady();
        }
        if(gamepad2.y) {
            grip();
        }


        //foundation mover control
        if(gamepad1.a) {
            grabFoundation();
        }

        if(gamepad1.b) {
            ungrabFoundation();
        }






        // AUTOMATED FLIP
        if(gamepad2.dpad_left) {
//            capstone.setPosition(capBetween);
        }

        //BACK IN
        if(gamepad2.dpad_right)
        {
//            capstone.setPosition(capDown);
        }








        /**
         *
         *
         * FLIP BACK
         *
         */





        /**
         *
         *
         * FLIP OUT
         *
         */




        /*
         * slide powers here
         */

        double slideMultiplier = 80; //todo:100
        if(gamepad2.right_stick_button) {
            slideMultiplier = 20;
        }


        double increment = gamepad2.right_stick_y * slideMultiplier;
        if(Math.abs(increment) > 5) {
            if(leftSlide.getCurrentPosition()+increment < -15 && rightSlide.getCurrentPosition()+increment < -15)
            {
                newSlideLeft = leftSlide.getCurrentPosition() + increment;
                newSlideRight = rightSlide.getCurrentPosition() + increment;

            }
            else{
                // stop slides from going down too much
            }

        }
        if(gamepad2.x) {
            oldSlideLeft = leftSlide.getCurrentPosition();
            oldSlideRight = rightSlide.getCurrentPosition();
            newSlideLeft = -25;
            newSlideRight = -25;
        }

        if(gamepad2.b) {
            oldSlideLeft = leftSlide.getCurrentPosition();
            oldSlideRight = rightSlide.getCurrentPosition();
            newSlideLeft = -14;
            newSlideRight = -14;
        }

        if(newSlideLeft - leftSlide.getTargetPosition() < 0 || newSlideRight - rightSlide.getTargetPosition() < 0) {
            leftSlide.setTargetPosition((int)(newSlideLeft));
            rightSlide.setTargetPosition((int)(newSlideRight));
            leftSlide.setPower(1);
            rightSlide.setPower(1);
        } else if(newSlideLeft - leftSlide.getTargetPosition() > 0 || newSlideRight - rightSlide.getTargetPosition() > 0) {
            leftSlide.setTargetPosition((int)(newSlideLeft));
            rightSlide.setTargetPosition((int)(newSlideRight));
            leftSlide.setPower(0.3);
            rightSlide.setPower(0.3);
        }
//
//        else {
//            leftSlide.setPower(0);
//            rightSlide.setPower(0);
//        }


        telemetry.addData("flipper pos", flipper.getPosition());
        telemetry.addData("gripper pos", gripper.getPosition());
        telemetry.addData("rotater pos", rotater.getPosition());
        telemetry.addData("left slide pos", leftSlide.getCurrentPosition());
        telemetry.addData("right slide pos", rightSlide.getCurrentPosition());
        telemetry.addData("left slide pid coefffs", leftSlide.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION).toString());
        telemetry.addLine(mecanumPowers());
        telemetry.addLine("-----------");
    }


    @SuppressLint("DefaultLocale")
    private String mecanumPowers() {
        return String.format(
                "\n" +
                        "(%.1f)---(%.1f)\n" +
                        "|   Front   |\n" +
                        "|             |\n" +
                        "|             |\n" +
                        "(%.1f)---(%.1f)\n"
                , leftFront.getPower(), rightFront.getPower(), leftRear.getPower(), rightRear.getPower());
    }



    public void setIntakePowers(double leftIntakePower, double rightIntakePower) {
        leftIntake.setPower(leftIntakePower);
        rightIntake.setPower(rightIntakePower);
    }


    public void stopIntake() { setIntakePowers(0,0);}


    // ready




    /**
     * foundation movement controls
     *
     *
     */

    public void grabFoundation() {
        leftSlam.setPosition(0.9);
        rightSlam.setPosition(0.1);
    }

    public void ungrabFoundation() {
        leftSlam.setPosition(0.1);
        rightSlam.setPosition(0.9);
    }


    /*
     * flipper movement controls
     */

    public void flip() {
        flipper.setPosition(flipperOut);
    }

    public void flipReady() {
        flipper.setPosition(flipperHome);
    }

    public void flipMid() {
        flipper.setPosition(flipperBetween);}


    /**
     * gripper controls
     */
    public void grip() {
        gripper.setPosition(GLOBALS.gripperHome);
    }

    public void gripReady() {
        gripper.setPosition(GLOBALS.gripperGrip);
    }



    /**
     * rotater movement controls
     */

    public void rotaterOut() {
        rotater.setPosition(rotaterOut);
    }

    public void rotaterReady() {
        rotater.setPosition(rotaterHome);
    }


}