package org.firstinspires.ftc.teamcode.code.hardware.statemachineproject.HelperClasses;

import android.os.SystemClock;
import com.qualcomm.hardware.bosch.BNO055IMU;
import net.frogbots.ftcopmodetunercommon.opmode.TunableOpMode;
import org.firstinspires.ftc.teamcode.code.hardware.statemachineproject.Hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.code.hardware.statemachineproject.Hardware.Intake;
import org.firstinspires.ftc.teamcode.code.hardware.statemachineproject.Hardware.Outtake;
import org.firstinspires.ftc.teamcode.code.hardware.statemachineproject.Hardware.Slide;
import org.firstinspires.ftc.teamcode.code.hardware.statemachineproject.RobotUtil.RobotMovement;
import org.firstinspires.ftc.teamcode.code.hardware.statemachineproject.RobotUtil.RobotPosition;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.ExpansionHubServo;
import org.openftc.revextensions2.RevBulkData;


import java.text.DecimalFormat;
import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.code.GLOBALS.*;

/**
 * this is the base state machine used for teleop and auto
 */
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
    public Outtake myOuttake;
    private DriveTrain myDriveTrain;


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
        allMotors.add(frontLeft);
        allMotors.add(frontRight);
        allMotors.add(backLeft);
        allMotors.add(backRight);

        // construct drivetrain
        myDriveTrain = new DriveTrain(this, frontLeft, frontRight, backLeft, backRight, master, slave, imu);




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
    }


    @Override
    public void start() {
        RobotPosition.initPose(myDriveTrain.getPoseEstimate(), this);
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

        long timeBeforeDataRead = System.currentTimeMillis();
        tp1.markStart();
        getRevBulkData();
        tp1.markEnd();

        long timeAfterDataRead = System.currentTimeMillis();
        telemetry.addData("Bulk data read time", timeAfterDataRead);


        currTimeMillis = SystemClock.uptimeMillis();
        elapsedMillisThisUpdate = (int)(currTimeMillis - lastLoopTime);
        lastLoopTime = currTimeMillis;




        // now updating the state machines starts

        tp2.markStart();
        myDriveTrain.updatee(); // updates drivetrain with non
        tp2.markEnd();


        tp3.markStart();
        myOuttake.update();
        tp3.markEnd();


        tp4.markStart();
        mySlide.update();
        tp4.markEnd();


        tp5.markStart();
        myIntake.update();
        tp5.markEnd();


        tp6.markStart();
        RobotPosition.giveMePose(myDriveTrain.getPoseEstimate());
        tp6.markEnd();


        // check for debug mode and tune pid gains
  //      tp7.markStart();
   //     debugMode();
    //    tp7.markStart();

   //     tp8.markStart();
   //     tuneSlidePID();
   //     tp8.markEnd();











        telemetry.addLine("profiler 1: " + tp1.getAverageTimePerUpdateMillis());
        telemetry.addLine("profiler 2: " + tp2.getAverageTimePerUpdateMillis());
        telemetry.addLine("profiler 3: " + tp3.getAverageTimePerUpdateMillis());
        telemetry.addLine("profiler 4: " + tp4.getAverageTimePerUpdateMillis());
        telemetry.addLine("profiler 5: " + tp5.getAverageTimePerUpdateMillis());
        telemetry.addLine("profiler 6: " + tp6.getAverageTimePerUpdateMillis());
        telemetry.addLine("profiler 7: " + tp7.getAverageTimePerUpdateMillis());
        telemetry.addLine("profiler 8: " + tp8.getAverageTimePerUpdateMillis());

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
            myDriveTrain.setDebugging(true);
            mySlide.setDebugging(true);
            myIntake.setDebugging(true);
            myOuttake.setDebugging(true);

    }




    public void addSpace() {
        telemetry.addLine("");
    }


}