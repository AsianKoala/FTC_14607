package org.firstinspires.ftc.teamcode.Teleop;
//BASED OFF OF AUTOMATED TELEOP DOES NOT ACCOUNT FOR NEW RESTRUCTURING

import android.annotation.SuppressLint;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import net.frogbots.ftcopmodetunercommon.opmode.TunableOpMode;

import org.openftc.revextensions2.ExpansionHubMotor;

import static org.firstinspires.ftc.teamcode.HelperClasses.GLOBALS.*;

import java.util.ArrayList;


@TeleOp(name = "main teleop")
public class HorseFlyTeleopExperimental extends TunableOpMode {

    /**
     * LIST OF TODOS
     * TODO: add integration with hardware class
     * TODO: add field centric drive that charlie coded
     * TODO: add more state machine stuff so its easier for the driver to use robot
     * TODO: add more stuff that makes it easier for driver to drive
     */

    private ExpansionHubMotor leftFront;
    private ExpansionHubMotor rightFront;
    private ExpansionHubMotor leftRear;
    private ExpansionHubMotor rightRear;
    private ExpansionHubMotor leftIntake;
    private ExpansionHubMotor rightIntake;
    private ExpansionHubMotor leftSlide;
    private ExpansionHubMotor rightSlide;
    private Servo flipper, gripper, rotater, leftSlam, rightSlam;

    private ArrayList<ExpansionHubMotor> driveMotors = new ArrayList<>();






    public final static long toMidTime = 450;
    public final static long liftTime = 200;
    public final static long toBackTime = 750;

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



        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        rightIntake.setDirection(DcMotor.Direction.REVERSE);
        leftSlide.setDirection(DcMotor.Direction.REVERSE);





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
         flipReady();
         rotaterReady();
         gripReady();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    // run until the end of the match (driver presses STOP)
    public void loop() {
        

        // tunable opmode vars
        

        
        
        
        

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
            motorPower = 0.95;
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
            flipper.setPosition(flipperBetween);
        }

        if(gamepad2.dpad_up) {
            flipper.setPosition(flipperBetween);
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






        /**
         *
         *
         * FLIP BACK
         *
         */


        switch(counter)
        {            
            //gripper let go first? 
            //task 1: lift up
            //need to test, this step should not be necesssary
            case 1:
                    gripper.setPosition(gripperGrip);
                    newSlideLeft = liftIncrementer;//height should not be hardcoded, fix after PID tuned
                    newSlideRight = liftIncrementer;
                
                //none of :
                    leftSlide.setTargetPosition((int)(newSlideLeft));
                    rightSlide.setTargetPosition((int)(newSlideRight));
                    leftSlide.setPower(1);
                    rightSlide.setPower(1);
                //this is necessary? coded later already    
                    
                    chime = System.currentTimeMillis();
                    counter++;
                    break;
 
            //rotate
            case 2:
                if(System.currentTimeMillis() - chime >= 100)//lift is super fast, can also rotate on the way up
                {
                    rotaterReady();
                    counter++;
                }
                break;
            //flip
            case 3:
                if(System.currentTimeMillis() - chime >= 750)//amount of time rotation takes
                {
                    flipper.setPosition(0.95);
                    chime = System.currentTimeMillis();
                    counter++;
                }

                break;
            //pseudo home lift
            case 4:
                leftSlide.setTargetPosition((int)psuedoHomer);//changed to pseudo home
                rightSlide.setTargetPosition((int)psuedoHomer);
                leftSlide.setPower(0.75);
                rightSlide.setPower(0.75);
                chime = System.currentTimeMillis();
                counter++;
                break;

            // grip to home
            case 5:
                if(System.currentTimeMillis() - chime >= 300) {
                    gripper.setPosition(gripperHome);
                }
                counter++;
                break;
           // lift to home 
            case 6:
                oldSlideLeft = leftSlide.getCurrentPosition();
                oldSlideRight = rightSlide.getCurrentPosition();
                newSlideLeft = psuedoHomer;
                newSlideRight = psuedoHomer;
                gripReady();
                counter = 0;
                break;
        }
        /**
         * 2.99
         * 0.042
         * 3.6
         *
         * 2.99
         * 0.042
         * 3.6
         * 2.99
         * 0.042
         * 3.6
         */


        /**
         *
         *
         * FLIP OUT
         *
         */

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

    /**
     * @return whether or not the intake motors are busy
     */


    public void setIntakePowers(double leftIntakePower, double rightIntakePower) {
        leftIntake.setPower(leftIntakePower);
        rightIntake.setPower(rightIntakePower);
    }




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
        gripper.setPosition(gripperGrip);
    }

    public void gripReady() {
        gripper.setPosition(gripperHome);
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
