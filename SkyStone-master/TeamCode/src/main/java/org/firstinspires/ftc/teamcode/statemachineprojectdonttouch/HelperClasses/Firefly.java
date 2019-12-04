package org.firstinspires.ftc.teamcode.statemachineprojectdonttouch.HelperClasses;

import android.annotation.SuppressLint;
import android.os.SystemClock;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import net.frogbots.ftcopmodetunercommon.opmode.TunableOpMode;
import org.firstinspires.ftc.teamcode.statemachineprojectdonttouch.Hardware.*;
import org.firstinspires.ftc.teamcode.statemachineprojectdonttouch.RobotUtil.RobotPosition;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.ExpansionHubServo;
import org.openftc.revextensions2.RevBulkData;


import java.text.DecimalFormat;
import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.HelperClasses.GLOBALS.*;

/**
 * this is the base state machine used for teleop and auto
 */
@TeleOp(name = "firefly class")
public class Firefly extends TunableOpMode {

    // rev objects
    RevBulkData masterData; // right hub
    RevBulkData slaveData; // left hub
    private ExpansionHubEx master;
    private ExpansionHubEx slave;
    private BNO055IMU imu;
    // wtf
    public DecimalFormat df = new DecimalFormat("#.00");

    // array for all motors
    private ArrayList<ExpansionHubMotor> allMotors = new ArrayList<>();


    // create hardware objects
    public Slide mySlide;
    public Intake myIntake;
    public DriveTrain myDriveTrain;
    public Outtake myOuttake;
    private DriveTrainTest myDriveTrainTest;
    public opencvSkystoneDetector myDetector;


    // used for debugging
    public long currTimeMillis = 0;





    /**
     * called when driver hits init
     */
    @Override
    public void init() {
        currTimeMillis = SystemClock.uptimeMillis();




        // time for mapping everything


        // map rev stuff
        master = hardwareMap.get(ExpansionHubEx.class, "master");
        slave = hardwareMap.get(ExpansionHubEx.class, "follower");
        imu = hardwareMap.get(BNO055IMU.class, "imu");


        ExpansionHubMotor frontLeft = hardwareMap.get(ExpansionHubMotor.class, "FL");
        ExpansionHubMotor frontRight = hardwareMap.get(ExpansionHubMotor.class, "FR");
        ExpansionHubMotor backLeft = hardwareMap.get(ExpansionHubMotor.class, "BL");
        ExpansionHubMotor backRight = hardwareMap.get(ExpansionHubMotor.class, "BR");

        // add all the motors to our array
     //   allMotors.add(frontLeft);
    //    allMotors.add(frontRight);
       // allMotors.add(backLeft);
   //    allMotors.add(backRight);

        // construct drivetrain
        myDriveTrainTest = new DriveTrainTest(frontLeft, frontRight, backLeft, backRight);




        // construct intake

        ExpansionHubMotor leftIntake = hardwareMap.get(ExpansionHubMotor.class, "leftIntake");
        ExpansionHubMotor rightIntake = hardwareMap.get(ExpansionHubMotor.class, "rightIntake");

        myIntake = new Intake(this, leftIntake, rightIntake);



        ExpansionHubMotor leftSlide = hardwareMap.get(ExpansionHubMotor.class, "leftSlide");
        ExpansionHubMotor rightSlide = hardwareMap.get(ExpansionHubMotor.class, "rightSlide");

        mySlide = new Slide(leftSlide, rightSlide);

        mySlide.setDebugging(false);


        ExpansionHubServo leftSlam = hardwareMap.get(ExpansionHubServo.class, "leftSlam");
        ExpansionHubServo rightSlam = hardwareMap.get(ExpansionHubServo.class, "rightSlam");
        ExpansionHubServo rotater = hardwareMap.get(ExpansionHubServo.class, "rotater");
        ExpansionHubServo flipper = hardwareMap.get(ExpansionHubServo.class, "flipper");
        ExpansionHubServo gripper = hardwareMap.get(ExpansionHubServo.class, "gripper");

        myOuttake = new Outtake(this, rotater, flipper, gripper, leftSlam, rightSlam);



        int id = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        myDetector = new opencvSkystoneDetector(this, id );


        myDetector.initCamera();
        myOuttake.init();
        myOuttake.update();
        mySlide.update();
        myIntake.update();

        getRevBulkData();

    }


    @Override
    public void init_loop() {
        currTimeMillis = SystemClock.uptimeMillis();
        getRevBulkData();
        mySlide.update();
        myDetector.update();
    }



    @Override
    public void start() {
        //RobotPosition.initPose(myDriveTrain.getPoseEstimate(), this);
    }

    /**
     * these are used as debugging loop time checks
     */

    private TimeProfiler tp1 = new TimeProfiler(1000);
    private TimeProfiler tp2 = new TimeProfiler(1000);
    private TimeProfiler tp3 = new TimeProfiler(1000);
    private TimeProfiler tp4 = new TimeProfiler(1000);
    private TimeProfiler tp5 = new TimeProfiler(1000);
    private TimeProfiler tp6 = new TimeProfiler(1000);
    private TimeProfiler tp7 = new TimeProfiler(1000);
    private TimeProfiler tp8 = new TimeProfiler(1000);
    private TimeProfiler tp9 = new TimeProfiler(1000);

    /**
     * time of last loop update in millis
     */
    private long lastLoopTime = 0;

    /**
     * amount of time elapsed this update in millis
     */
    public int elapsedMillisThisUpdate = 0;



    private TimeProfiler timeProfiler = new TimeProfiler(300);
    public static double updatesPerSecond = 1000;

    @Override
    public void loop() {
        /**
         * most of this stuff is just state machine updating + telemetry
         */
        timeProfiler.markEnd();
        timeProfiler.markStart();


        updatesPerSecond = 1000/timeProfiler.getAverageTimePerUpdateMillis();
        telemetry.addLine("UPS: " + updatesPerSecond);

        long timeBefore = SystemClock.uptimeMillis();
        tp2.markStart();
        //get all the bulk data
        getRevBulkData();
        tp2.markEnd();

        long timeAfter = SystemClock.uptimeMillis();
        telemetry.addData("Bulk data time: ", (timeAfter-timeBefore));


        currTimeMillis = SystemClock.uptimeMillis();
        elapsedMillisThisUpdate = (int)(currTimeMillis - lastLoopTime);
        lastLoopTime = currTimeMillis;




        // now updating the state machines starts

        tp1.markStart();
        myDriveTrainTest.driveMecanum(movementX, movementY, movementTurn); // applies movementX etc. to drive motor powers
        tp1.markEnd();

       // tp3.markStart();
      //  myDriveTrain.update(); // updates roadrunner pose using motor encoder values
      //  tp3.markEnd();

        tp4.markStart();
     //   RobotPosition.giveMePose(myDriveTrain.getPoseEstimate()); // updates worldXPos etc. from roadrunner pose
        tp4.markEnd();

        /**
         * referencing above ^
         * the reason we have to do myDriveTrain.update(); (roadrunner super method) and do
         * RobotPosition.giveMePose is because we convert the pose estimate obtained through rr into vars
         * that are used for everything besides roadrunner movement methods.
         * robot position is actually pretty useless since we dont use any other movement methods
         * **** ROBOT POSITION DOES NOT SET ANYTHING, RATHER CONVERTS RR POSE TO OUR VALUES ****
         */


        tp5.markStart();
        myOuttake.update();
        tp5.markEnd();


        tp6.markStart();
        mySlide.update();
        tp6.markEnd();


        tp7.markStart();
        myIntake.update();
        tp7.markEnd();






        // check for debug mode and tune pid gains
  //      tp7.markStart();
   //     debugMode();
    //    tp7.markStart();



        // in millis
        telemetry.addLine("profiler 1: " + tp1.getAverageTimePerUpdateMillis());
        telemetry.addLine("profiler 2: " + tp2.getAverageTimePerUpdateMillis());
        telemetry.addLine("profiler 3: " + tp3.getAverageTimePerUpdateMillis());
        telemetry.addLine("profiler 4: " + tp4.getAverageTimePerUpdateMillis());
        telemetry.addLine("profiler 5: " + tp5.getAverageTimePerUpdateMillis());
        telemetry.addLine("profiler 6: " + tp6.getAverageTimePerUpdateMillis());
        telemetry.addLine("profiler 7: " + tp7.getAverageTimePerUpdateMillis());
        telemetry.addLine("profiler 8: " + tp8.getAverageTimePerUpdateMillis());
        telemetry.addLine(mecanumPowers());
        telemetry.update();

    }


    @SuppressLint("DefaultLocale")
    public String mecanumPowers() {
        return String.format(
                "\n" +
                        "(%.1f)---(%.1f)\n" +
                        "|   Front      |\n" +
                        "|                  |\n" +
                        "|                  |\n" +
                        "(%.1f)---(%.1f)\n"
                , myDriveTrainTest.frontLeft.getPower(), myDriveTrainTest.frontRight.getPower(), myDriveTrainTest.backLeft.getPower(), myDriveTrainTest.backRight.getPower());
    }




    /**
     * teleop user control
     */
    public void teleopDrivetrainControl() {
        double scale = 0.8; // btich im not changing this to 1
        if(gamepad1.left_bumper) {
            scale = 0.5;
        }
        if(gamepad1.right_bumper) {
            scale = 0.25;
        }

        double threshold = 0.157;

        movementY =  Math.abs(-gamepad1.left_stick_y) > threshold ? -gamepad1.left_stick_y * scale : 0;
        movementX = Math.abs(gamepad1.left_stick_x) > threshold ? gamepad1.left_stick_x * scale : 0;
        movementTurn = Math.abs(gamepad1.right_stick_x) > threshold ? gamepad1.right_stick_x * scale : 0;
    }


    /**
     * gets all the data from the expansion hubs in one command
     */
    public void getRevBulkData() {
        RevBulkData newMasterData;

        try {
            newMasterData = master.getBulkInputData();
            if(newMasterData != null) {
                masterData = newMasterData;
            }
        }
        catch(Exception e) {
            // dont do anything if we get exception
        }


        RevBulkData newSlaveData;

        try {
            newSlaveData = slave.getBulkInputData();
            if(newSlaveData != null) {
                slaveData = newSlaveData;
            }
        }

        catch(Exception e) {}
    }




    // report position



    public double getXPos() { return RobotPosition.worldXPos; }
    public double getYPos() { return RobotPosition.worldYPos; }
    public double getHeadingRad() {return RobotPosition.worldHeadingRad; }
    public double getDegrees() { return Math.toDegrees(RobotPosition.worldHeadingRad); }



    private void tuneSlidePID() {

            double p = getInt("P");
            double i = getInt("I");
            double d = getInt("D");
            mySlide.setPIDCoeffs(p, i, d);
    }


    private void debugMode() {
          //  myDriveTrain.setDebugging(true);
            mySlide.setDebugging(true);
            myIntake.setDebugging(true);
            myOuttake.setDebugging(true);
    }




    public void addSpace() {
        telemetry.addLine("");
    }


}