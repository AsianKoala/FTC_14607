package org.firstinspires.ftc.teamcode.code.Teleop.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.code.Firefly;

import java.util.ArrayList;


@TeleOp(name = "hosrefly automated")
public class BugFlyTeleop extends OpMode {

    /**
     * LIST OF TODOS
     * TODO: add integration with hardware class
     * TODO: add field centric drive that charlie coded
     * TODO: add more state machine stuff so its easier for the driver to use robot
     * TODO: add more stuff that makes it easier for driver to drive
     */

    private Firefly robot;



    private long time = 0;
    private int count = 0;

    private long chime = 0;
    private int counter = 0;

    private final long toMidTime = 450;
    private final long liftTime = 200;
    private final long toBackTime = 750;

    private final long toLiftTimeTo = 400;
    private final long toBackTimeTo = 700;

    private final int liftIncrement = -200;
    private final int liftIncrementer = -500;


    private double oldSlideLeft = 0;
    private double oldSlideRight = 0;
    private double newSlideLeft = 0;
    private double newSlideRight = 0;



    @Override
    public void init() {

        robot = new Firefly(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    // run until the end of the match (driver presses STOP)
    public void loop() {



        /**
         *
         * INTAKE CONTROL
         *
         */

        double leftIntakePower = gamepad2.left_stick_y - gamepad2.left_stick_x;
        double rightIntakePower = gamepad2.left_stick_y + gamepad2.left_stick_x;
        if(Math.abs(leftIntakePower) < 0.1 || Math.abs(rightIntakePower) < 0.1) {
            robot.leftIntake.setPower(0);
            robot.rightIntake.setPower(0);
        }else {
            robot.leftIntake.setPower( 0.5 * -leftIntakePower);
            robot.rightIntake.setPower( 0.5 * -rightIntakePower);
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
            robot.frontRight.setPower(motorPower * (((-gamepad1.left_stick_y) + (gamepad1.left_stick_x)) + -gamepad1.right_stick_x));
            robot.backLeft.setPower(motorPower * (((-gamepad1.left_stick_y) + (-gamepad1.left_stick_x)) + gamepad1.right_stick_x));
            robot.frontLeft.setPower(motorPower * (((-gamepad1.left_stick_y) + (gamepad1.left_stick_x)) + gamepad1.right_stick_x));
            robot.backRight.setPower(motorPower * (((-gamepad1.left_stick_y) + (-gamepad1.left_stick_x)) + -gamepad1.right_stick_x));
        }

        else
        {
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);
        }











        // flipper arm control

        if(gamepad2.dpad_down) {
            flipper.setPosition(.6);
        }

        if(gamepad2.dpad_up) {
            flipper.setPosition((0.25+0.95)/2);
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

        // AUTOMATED FLIP
        if(gamepad2.dpad_left) {
            count = 1;
        }

        //BACK IN
        if(gamepad2.dpad_right)
        {
            counter = 1;
        }






        switch(counter)
        {
                //task 1: lift up
            case 1:
                    gripper.setPosition(gripperGrip);
                    newSlideLeft = liftIncrementer;
                    newSlideRight = liftIncrementer;
                    leftSlide.setTargetPosition((int)(newSlideLeft));
                    rightSlide.setTargetPosition((int)(newSlideRight));
                    leftSlide.setPower(1);
                    rightSlide.setPower(1);
                    chime = System.currentTimeMillis();
                    counter++;

                break;
            //task 3: flip back
            case 2:
                if(System.currentTimeMillis() - chime >= toLiftTimeTo)
                {
                    flipper.setPosition(0.95);
                    chime = System.currentTimeMillis();
                    counter++;
                }

                break;
            //rotate around
            case 3:
                if(System.currentTimeMillis() - chime >= toBackTimeTo)
                {
                    rotaterReady();
                    counter++;
                }
                break;
            case 4:
                leftSlide.setTargetPosition((int)(-25.0/2));
                rightSlide.setTargetPosition((int)(-25.0/2));
                leftSlide.setPower(0.75);
                rightSlide.setPower(0.75);
                chime = System.currentTimeMillis();
                counter++;
                break;

            // grip to home
            case 5:
                if(System.currentTimeMillis() - chime >= 500) {
                    gripper.setPosition(gripperHome);
                }

                counter = 0;
                break;
        }




        switch(count)
        {
            //task 1: flip to center
            case 1:
                flipper.setPosition(0.6);
                time = System.currentTimeMillis();
                count++;
                break;
            //task 2: lift up
            case 2:
                if(System.currentTimeMillis() - time >= toMidTime)
                {
                    newSlideLeft = liftIncrement;
                    newSlideRight = liftIncrement;
                    leftSlide.setTargetPosition((int)(newSlideLeft));
                    rightSlide.setTargetPosition((int)(newSlideRight));
                    leftSlide.setPower(1);
                    rightSlide.setPower(1);
                    time = System.currentTimeMillis();
                    count++;
                }
                break;
            //task 3: flip back
            case 3:
                if(System.currentTimeMillis() - time >= liftTime)
                {
                    flipper.setPosition(0.25);
                    time = System.currentTimeMillis();
                    count++;
                }

                break;

        /*    case 4:
                if(System.currentTimeMillis() - time >= 100) {
                    flipper.setPosition(flipperBetweenBetween);
                    time = System.currentTimeMillis();
                    count++;
                }*/

            //rotate around
            case 4:
                if(System.currentTimeMillis() - time >= toBackTime)
                {
                    rotaterOut();
                    count = 0;
                }
                break;
        }


        /**
         * slide powers here
         */

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


        telemetry.addData("flipper pos", flipper.getPosition());
        telemetry.addData("gripper pos", gripper.getPosition());
        telemetry.addData("rotater pos", rotater.getPosition());
        telemetry.addData("left slide pos", leftSlide.getCurrentPosition());
        telemetry.addData("right slide pos", rightSlide.getCurrentPosition());
        telemetry.addData("left slide pid coefffs", leftSlide.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION).toString());
        telemetry.addData("right slide pid coeffs", rightSlide.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION).toString());

    }




    /**
     * @return whether or not the intake motors are busy
     */

    public boolean intakeBusy() {
        return leftIntake.isBusy() || rightIntake.isBusy();}

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

    public boolean isOuttakeReady() {
        return rotater.getPosition() == rotaterHome;}

}