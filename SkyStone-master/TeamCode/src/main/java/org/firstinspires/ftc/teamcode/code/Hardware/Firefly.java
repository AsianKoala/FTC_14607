package org.firstinspires.ftc.teamcode.code.Hardware;

import android.os.SystemClock;
import com.acmerobotics.roadrunner.drive.Drive;
import com.qualcomm.hardware.bosch.BNO055IMU;
import net.frogbots.ftcopmodetunercommon.opmode.TunableOpMode;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

import java.lang.reflect.Array;
import java.util.ArrayList;

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

    // array for all motors
    private ArrayList<ExpansionHubMotor> allMotors = new ArrayList<>();


    // create hardware objects
    public Slide mySlide;
    public Intake myIntake;
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
        myDriveTrain = new DriveTrain(frontLeft, frontRight, backLeft, backRight, master, slave, imu);




        // construct intake

        ExpansionHubMotor leftIntake = hardwareMap.get(ExpansionHubMotor.class, "leftIntake");
        ExpansionHubMotor rightIntake = hardwareMap.get(ExpansionHubMotor.class, "rightIntake");

        myIntake = new Intake(leftIntake, rightIntake);



        ExpansionHubMotor leftSlide = hardwareMap.get(ExpansionHubMotor.class, "leftSlide");
        ExpansionHubMotor rightSlide = hardwareMap.get(ExpansionHubMotor.class, "rightSlide");

        mySlide = new Slide(leftSlide, rightSlide);

        mySlide.setDebugging(false);





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
        telemetry.addData("Bulk data time", timeAfterDataRead);


        currTimeMillis = SystemClock.uptimeMillis();
        elapsedMillisThisUpdate = (int)(currTimeMillis - lastLoopTime);
        lastLoopTime = currTimeMillis;


        // now updating the state machines starts

        // intake update
        tp2.markStart();
        myIntake.update();
        tp2.markEnd();


        // slide update
        tp3.markStart();
        mySlide.update();
        tp3.markEnd();


        telemetry.addLine("profiler 1: " + tp1.getAverageTimePerUpdateMillis());
        telemetry.addLine("profiler 2: " + tp2.getAverageTimePerUpdateMillis());
        telemetry.addLine("profiler 3: " + tp3.getAverageTimePerUpdateMillis());


    }


    /**
     * teleop user control
      */
    public void teleopDrivetrainControl() {
        double scale = 0.8;
        if(gamepad1.left_bumper) {
            scale = 0.5;
        }
        if(gamepad1.right_bumper) {
            scale = 0.25;
        }
        myDriveTrain.driveMecanum(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, true, scale);
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


    public double getLeftSlideCurrentPosition() {
        return slaveData.getMotorCurrentPosition(3);
    }

    public double getRightSlideCurrentPosition() {
        return masterData.getMotorCurrentPosition(3);
    }

    public double getCurrentIntakeVelocty() {
        return (masterData.getMotorVelocity(2) + slaveData.getMotorVelocity(2))/2.0;
    }


}