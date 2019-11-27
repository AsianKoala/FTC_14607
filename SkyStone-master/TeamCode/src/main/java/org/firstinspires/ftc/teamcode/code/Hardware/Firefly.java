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

    }


    @Override
    public void init_loop() {
        currTimeMillis = SystemClock.uptimeMillis();

        mySlide.update();
    }


    @Override
    public void loop() {

    }



}