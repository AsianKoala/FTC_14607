

package org.firstinspires.ftc.teamcode.code.Teleop.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;


@TeleOp(name = "basiceleopdrve")
public class HouseFlyTeleop extends OpMode {

    /**
     * LIST OF TODOS
     * TODO: add integration with hardware class
     * TODO: add field centric drive that charlie coded
     * TODO: add more state machine stuff so its easier for the driver to use robot
     * TODO: add more stuff that makes it easier for driver to drive
     */

    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftRear;
    private DcMotor rightRear;
    private DcMotor leftIntake;
    private DcMotor rightIntake;
    private DcMotorEx leftSlide;
    private DcMotorEx rightSlide;
    private Servo flipper, gripper, rotater, leftSlam, rightSlam;

    private ArrayList<DcMotor> driveMotors = new ArrayList<>();




    private final double flipperHome =  0.95;
    private final double flipperOut = 0.25;
    private final double flipperBetween = (flipperHome + flipperOut)/2;
    private final double rotaterHome = 0.279;
    private final double rotaterOut = 0.95;
    private final double gripperHome = 0.41;
    private final double gripperGrip = 0.2;

    private double oldSlideLeft = 0;
    private double oldSlideRight = 0;
    private double newSlideLeft = 0;
    private double newSlideRight = 0;



    @Override
    public void init() {

        leftFront = hardwareMap.get(DcMotor.class, "FL");
        leftRear = hardwareMap.get(DcMotor.class, "BL");
        rightRear = hardwareMap.get(DcMotor.class, "FR");
        rightFront = hardwareMap.get(DcMotor.class, "BR");
        leftIntake = hardwareMap.get(DcMotor.class, "leftIntake");
        rightIntake = hardwareMap.get(DcMotor.class, "rightIntake");
        leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");

        gripper = hardwareMap.get(Servo.class, "gripper");
        flipper = hardwareMap.get(Servo.class, "flipper");
        rotater = hardwareMap.get(Servo.class, "rotater");
        leftSlam = hardwareMap.get(Servo.class, "leftSlam");
        rightSlam = hardwareMap.get(Servo.class, "rightSlam");



        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        rightIntake.setDirection(DcMotor.Direction.REVERSE);
        leftSlide.setDirection(DcMotor.Direction.REVERSE);





        leftSlide.setTargetPosition(0);
        rightSlide.setTargetPosition(0);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        driveMotors.add(leftRear);
        driveMotors.add(leftFront);
        driveMotors.add(rightFront);
        driveMotors.add(rightRear);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    // run until the end of the match (driver presses STOP)
    public void loop() {

        /**
         *
         * SLIED CONTROL
         *
         */



//        if(gamepad2.right_stick_button) {
//            newSlideLeft = oldSlideLeft;
//            newSlideRight = oldSlideRight;
//        }

        double increment = gamepad2.right_stick_y * 100;

        if(Math.abs(increment) > 25) {
            newSlideLeft = leftSlide.getCurrentPosition() + increment;
            newSlideRight = rightSlide.getCurrentPosition() + increment;
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
            newSlideLeft = -25.0/2;
            newSlideRight = -25.0/2;
        }

        if(Math.abs(newSlideLeft - leftSlide.getCurrentPosition()) > 10 || Math.abs(newSlideRight - rightSlide.getCurrentPosition()) > 10) {
            leftSlide.setTargetPosition((int)(newSlideLeft));
            rightSlide.setTargetPosition((int)(newSlideRight));
            leftSlide.setPower(1);
            rightSlide.setPower(1);
        }

        else {
            leftSlide.setPower(0);
            rightSlide.setPower(0);
        }







        /**
         *
         * INTAKE CONTROL
         *
         */

        double leftIntakePower = gamepad2.left_stick_y - gamepad2.left_stick_x;
        double rightIntakePower = gamepad2.left_stick_y + gamepad2.left_stick_x;
        if(Math.abs(leftIntakePower) < 0.1 || Math.abs(rightIntakePower) < 0.1) {
            leftIntake.setPower(0);
            rightIntake.setPower(0);
        }else {
            leftIntake.setPower( 0.5 * -leftIntakePower);
            rightIntake.setPower( 0.5 * -rightIntakePower);
        }


        /**
         *
         *
         * DRIVE MOTORS CONTROL
         *
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











        // flipper arm control

        if(gamepad2.dpad_down) {
            flipper.setPosition(.6);
        }

        if(gamepad2.dpad_up) {
            flipper.setPosition((0.6+0.95)/2);
        }

        if(gamepad2.right_trigger > 0.5) {
            flipper.setPosition(0.95);
        }

        if(gamepad2.left_trigger > 0.5) {
            flipper.setPosition(0.25);
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
            grip();
        }
        if(gamepad2.y) {
            gripReady();
        }




        //foundation mover control
        if(gamepad1.a) {
            grabFoundation();
        }

        if(gamepad1.b) {
            ungrabFoundation();
        }


        telemetry.addData("flipper pos", flipper.getPosition());
        telemetry.addData("gripper pos", gripper.getPosition());
        telemetry.addData("rotater pos", rotater.getPosition());
        telemetry.addData("left slide pos", leftSlide.getCurrentPosition());
        telemetry.addData("right slide pos", rightSlide.getCurrentPosition());

    }




    /**
     * @return whether or not the intake motors are busy
     */

    public boolean intakeBusy() { return leftIntake.isBusy() || rightIntake.isBusy();}

    public void setIntakePowers(double leftIntakePower, double rightIntakePower) {
        leftIntake.setPower(leftIntakePower);
        rightIntake.setPower(rightIntakePower);
    }


    public void stopIntake() { setIntakePowers(0,0);}


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


    /**
     *
     *
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


    public boolean isFlipperReady() {
        return flipper.getPosition() == flipperHome;
    }

    public boolean isFlipperFlipped() {
        return flipper.getPortNumber() == flipperOut;
    }



    /**
     * gripper controls
     */

    public void grip() {
        gripper.setPosition(gripperGrip);
    }

    public void gripReady() {
        gripper.setPosition(gripperHome);
    }

    public boolean isGripReady() {
        return gripper.getPosition() == gripperHome;
    }

    public boolean isGripped() {
        return gripper.getPosition() == gripperGrip;
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

    public boolean isOuttaked() {
        return rotater.getPosition() == rotaterOut;
    }

    public boolean isOuttakeReady() { return rotater.getPosition() == rotaterHome;}

}