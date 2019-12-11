package org.firstinspires.ftc.teamcode.statemachineprojectdonttouch.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.statemachineprojectdonttouch.HelperClasses.Firefly;
import org.firstinspires.ftc.teamcode.statemachineprojectdonttouch.HelperClasses.TimeProfiler;

import static org.firstinspires.ftc.teamcode.HelperClasses.GLOBALS.*;
import static org.firstinspires.ftc.teamcode.statemachineprojectdonttouch.RobotUtil.RobotPosition.*;

@TeleOp(name = "dont click me owo", group = "teleop")
public class FireflyTeleop extends Firefly {

    /**
     * variable declaration
     */


    private int count = 0;
    private double time = 0;
    private double chime = 0;
    private int counter = 0;



    @Override
    public void init() {
        super.init();
        debugMode(true);
    }

    @Override
    public void init_loop() {
        super.init_loop();
    }

    @Override
    public void start() {
        super.start();
        giveMePose(new Pose2d(24,24, Math.toRadians(90)));
    }


    private TimeProfiler tp1 = new TimeProfiler(1000);
    private TimeProfiler tp2 = new TimeProfiler(1000);
    private TimeProfiler tp3 = new TimeProfiler(1000);
    private TimeProfiler tp4 = new TimeProfiler(1000);
    private TimeProfiler tp5 = new TimeProfiler(1000);
    private TimeProfiler tp6 = new TimeProfiler(1000);
    private TimeProfiler tp7 = new TimeProfiler(1000);
    private TimeProfiler tp8 = new TimeProfiler(1000);
    private TimeProfiler tp9 = new TimeProfiler(1000);

    @Override
    public void loop() {


        tp1.markStart();
        super.loop();
        tp1.markEnd();


        tp2.markStart();
        teleopDrivetrainControl();
        tp2.markEnd();


        tp3.markStart();
        slideControl();
        tp3.markEnd();


        tp4.markStart();
        intakeControl();
        tp4.markEnd();


        tp5.markStart();
        servoControl();
        tp5.markEnd();


        tp6.markStart();
        gamepadTelem();
        tp6.markEnd();



        addSpace();
        telemetry.addLine("-------------- FIREFLY TELEOP TELEMETRY -----------------");
        telemetry.addLine("tOp profiler 1: " + tp1.getAverageTimePerUpdateMillis());
        telemetry.addLine("tOp profiler 2: " + tp2.getAverageTimePerUpdateMillis());
        telemetry.addLine("tOp profiler 3: " + tp3.getAverageTimePerUpdateMillis());
        telemetry.addLine("tOp profiler 4: " + tp4.getAverageTimePerUpdateMillis());
        telemetry.addLine("tOp profiler 5: " + tp5.getAverageTimePerUpdateMillis());
        telemetry.addLine("tOp profiler 6: " + tp6.getAverageTimePerUpdateMillis());
        telemetry.addLine("tOp profiler 7: " + tp7.getAverageTimePerUpdateMillis());
        telemetry.addLine("tOp profiler 8: " + tp8.getAverageTimePerUpdateMillis());
        telemetry.addLine("tOp profiler 9: " + tp9.getAverageTimePerUpdateMillis());

    }

// ready
    /**
     * teleop user control
     */
    public void teleopDrivetrainControl() {
        double scale = Math.abs(0.8 + (gamepad1.left_bumper ? -0.3 : gamepad1.right_bumper ? -0.6 : 0));

        double threshold = 0.157;

        movementY =  Math.abs(-gamepad1.left_stick_y) > threshold ? -gamepad1.left_stick_y * scale : 0;
        movementX = Math.abs(gamepad1.left_stick_x) > threshold ? gamepad1.left_stick_x * scale : 0;
        movementTurn = Math.abs(gamepad1.right_stick_x) > threshold ? gamepad1.right_stick_x * scale : 0;
    }


// ready
    public void gamepadTelem() {
        telemetry.addData("gpad 1 ls y: ", gamepad1.left_stick_y);
        telemetry.addData("gpad 1 ls x: ", gamepad1.left_stick_x);
        telemetry.addData("gpad 1 rs x: ", gamepad1.right_stick_x);
    }



    private void servoControl() {

        if(gamepad2.dpad_down) {
            myOuttake.flipMid();
        }

        if(gamepad2.dpad_up) {
            myOuttake.flipMid();
        }

        if(gamepad2.right_trigger > 0.5) {
            myOuttake.flipReady();
        }

        if(gamepad2.left_trigger > 0.5) {
            myOuttake.flipOut();
        }



        if(gamepad2.left_bumper) {
            myOuttake.rotaterOut();
        }

        if(gamepad2.right_bumper) {
            myOuttake.rotaterReady();
        }



        if(gamepad2.a) {
            myOuttake.grip();
        }

        if(gamepad2.y) {
            myOuttake.gripReady();
        }



        if(gamepad1.a) {
            myOuttake.grabFoundation();
        }

        if(gamepad1.b) {
            myOuttake.ungrabFoundation();
        }
    }




    /**
     * our slide control
     */
    private void slideControl() {

        // custom control

        double increment = gamepad2.right_stick_y * 100;

        if(Math.abs(increment) > 25) { mySlide.goCustom(increment, true); }
        if(gamepad2.x) { mySlide.goPsuedoHome(); }
        if(gamepad2.b) { mySlide.goPsuedoHome(); }





        // automated slide control

        if(gamepad2.dpad_left) {
            count = 1;
        }

        if(gamepad2.dpad_right) {
            counter = 1;
        }


        switch(count) {

            // task 1: flip to center
            case 1:
                myOuttake.flipMid();
                time = System.currentTimeMillis();
                count ++;
                break;

            // task 2: lift up
            case 2:
                if(System.currentTimeMillis() - time >= toMidTime){
                    mySlide.goCustom(liftIncrement, false);

                    time = System.currentTimeMillis();
                    count++;
                }
                break;

            case 3:
                // task 3: flip black
                if(System.currentTimeMillis() - time >= liftTime) {
                    myOuttake.flipOut();
                    time = System.currentTimeMillis();
                    count++;
                }
                break;

            case 4:
                // task 4: rotate around
                if(System.currentTimeMillis() - time >= toBackTime) {
                    myOuttake.rotaterOut();
                    count = 0;
                }
                break;


        }


        switch(counter) {
            case 1:
                // task 1: lift up a bit
                myOuttake.gripReady();
                myOuttake.rotaterReady();
                mySlide.goCustom(liftIncrementer, false);

                chime = System.currentTimeMillis();
                counter++;
                break;


            case 2:
                // task 2: flip back
                if(System.currentTimeMillis() - chime >= 750) {
                    myOuttake.flipReady();
                    chime = System.currentTimeMillis();
                    counter++;
                }
                break;

            case 3:
                mySlide.goPsuedoHome();

                chime = System.currentTimeMillis();
                counter = 0;
                break;
        }
    }





    /**
     * our intake control
     */
    private void intakeControl() {

        double leftIntakePower = gamepad2.left_stick_y - gamepad2.left_stick_x;
        double rightIntakePower = gamepad2.left_stick_y + gamepad2.left_stick_x;

        myIntake.manualControl( (Math.abs(leftIntakePower) < 0.1 ? 0 : 0.5 * -leftIntakePower),
                (Math.abs(rightIntakePower) < 0.1 ? 0 : 0.5 * -rightIntakePower));

    }

}