package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.TankDrive;
import com.acmerobotics.roadrunner.followers.TankPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.followers.GVFFollower;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.acmerobotics.roadrunner.trajectory.constraints.TankConstraints;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import java.util.List;
import java.util.Arrays;

import org.firstinspires.ftc.teamcode.roadrunner.util.LynxOptimizedI2cFactory;
import org.jetbrains.annotations.NotNull;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;
import org.openftc.revextensions2.RevExtensions2;

/**
 * Created by charliewu on 9/20/18.
 */

@Config
public class HardwareDragonflyMP extends TankDrive {
    /* Public OpMode members. */
    public ExpansionHubMotor fl   = null;
    public ExpansionHubMotor  fr  = null;
    public ExpansionHubMotor bl   = null;
    public ExpansionHubMotor  br  = null;

    public DcMotorEx arm   = null;
    public DcMotorEx cascade   = null;
    public ExpansionHubMotor lift   = null;

    public DcMotor intake_motor = null;

    public CRServo intake = null;
    public CRServo intake2 = null;

    public Servo intakeDoor = null;
    public Servo hangRelease = null;
    public Servo markerDeployer = null;
    public Servo hookRelease = null;

    public BNO055IMU revIMU = null;

    private ExpansionHubEx driveHub;
    private List<ExpansionHubMotor> motors, leftMotors, rightMotors;


    //ROBOT CONFIG CONSTANTS
    public static int ARM_LOWERED_VAL = 0;
    public static int ARM_LEVEL_VAL = 743;
    public static int ARM_VERTICAL_VAL = 1905;
    public static int ARM_TRANSITION_VAL = 1000; //1200
    public static int ARM_PARK_VAL = 900;

    public static int ARM_VERTICAL_PLUS_VAL = 1905+200;

    public static int ARM_PICK_VAL = 475;

    public static int ARM_LEFT_GOLD_VAL = 205; //232 too high and 218
    public static int ARM_CENTER_GOLD_VAL = 162; //162
    public static int ARM_RIGHT_GOLD_VAL = 240; //232

    public static int ARM_MARKER_DEPLOY_VAL = 560; //675

    public static int LIFT_DOWN_VAL = 0;
    public static int LIFT_MAX_VAL = -28866;

    public static int LIFT_SMALL_LIFT_VAL = -1250 -233;
    public static int LIFT_FULL_LIFT_VAL = -700 -233;

    public static int LIFT_HOOK_VAL = -3218; //-24273 -2985
    public static int LIFT_DETATCH_VAL = -1792 -233; //-18516; //-19000

    public static int CASCADE_IN_VAL = 0; //0
    public static int CASCADE_MAX_VAL = -2250; //-4500
    public static int CASCADE_SCORE_DEFAULT_VAL = -350; //-560 540 545 555 278 -300 was good

    public static int CASCADE_SCORE_FAR_VAL = -380; //-760

    //TODO: ^^ min values, need tuning at practice fields
    //start with cascade fully retracted


    public static int CASCADE_LEFT_GOLD_EXTEND_VAL = -1250; //-2200 2500
    public static int CASCADE_CENTER_GOLD_EXTEND_VAL = -950; //-1900
    public static int CASCADE_RIGHT_GOLD_EXTEND_VAL = -1350; //-2500 2700
    public static int CASCADE_MARKER_EXTEND_VAL = -2250; //-4500

    public static int TURN_OUT_DELATCH_VAL = -30; //degrees
    public static int TURN_OUT_RESET_VAL = 0; //degrees

    public static int TURN_CENTER_GOLD_MINADJUST = 5; //degrees

    public static int DRIVE_DEPOT_PARK_MOVE_1 = 12; //inches
    public static int DRIVE_DEPOT_PARK_TURN_1 = -85; //degrees -90 works
    public static int DRIVE_DEPOT_PARK_MOVE_2 = 45; //inches //50
    public static int DRIVE_DEPOT_PARK_TURN_2 = -135; //degrees
    public static int DRIVE_DEPOT_PARK_MOVE_3 = 5; //inches //12

    //unused
    public static int TURN_OUT_DRIVE_PARK_VAL_1 = -40; //degrees
    public static int FORWARD_MOVE_PARK_VAL_1 = 19; //inches //22 before
    public static int TURN_OUT_DRIVE_PARK_VAL_2 = -110; //degrees //-100
    public static int FORWARD_MOVE_PARK_VAL_2 = 12; //inches //10
    // end unused

    public static int DRIVE_DEPOT_CLAIM_MOVE_1 = 14; //inches
    public static int DRIVE_DEPOT_CLAIM_TURN_1 = -80; //degrees
    public static int DRIVE_DEPOT_CLAIM_MOVE_2 = 40; //inches //45
    public static int DRIVE_DEPOT_CLAIM_TURN_2 = -130; //degrees
    public static int DRIVE_DEPOT_CLAIM_MOVE_3 = 5; //inches //5 works
    public static int DRIVE_DEPOT_CLAIM_MOVE_SCORE_ADJ = 3;

    public static int FORWARD_MOVE_SAMPLING_VAL = 13; //inches //15
    public static int FORWARD_MOVE_PUSH_SAMPLING_VAL = 10; //inches //15
    public static int FORWARD_MOVE_MARKER_VAL_EXTRA = 17; //inches //19 before
    public static int BACKWARDS_MOVE_SAMPLING_VAL = 6; //inches
    public static int TURN_LEFT_GOLD_VAL = -35; //degrees
    public static int TURN_RIGHT_GOLD_VAL = 42; //degrees
    public static int FORWARD_MOVE_GOLD_CENTER_PUSH_VAL = 18; //inches
    public static int FORWARD_MOVE_GOLD_SIDE_PUSH_VAL = 20; //inches

            //END ROBOT CONFIG CONSTANTS

//
//    public Servo sv1 = null;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    //MP settings
    public static PIDCoefficients DISPLACEMENT_PID = new PIDCoefficients(0, 0, 0);
    public static PIDCoefficients CROSS_TRACK_PID = new PIDCoefficients(0, 0, 0);

    public static PIDCoefficients STANDARD_PID = new PIDCoefficients(10, 3, 1);


    private DriveConstraints constraints;
    private TrajectoryFollower follower;

    /* Constructor */
    public HardwareDragonflyMP(){
        super(DriveConstants.TRACK_WIDTH);

//        this.init(hwMap);
//
//        constraints = new TankConstraints(DriveConstants.BASE_CONSTRAINTS, DriveConstants.TRACK_WIDTH);
//        follower = new TankPIDVAFollower(this, DISPLACEMENT_PID, CROSS_TRACK_PID,
//                DriveConstants.kV, DriveConstants.kA, DriveConstants.kStatic);
//
////        driveHub = hwMap.get(ExpansionHubEx.class, "driveHub");
//
//        motors = Arrays.asList(fl, bl, br, fr);
//        leftMotors = Arrays.asList(fl, bl);
//        rightMotors = Arrays.asList(fr, br);
//
//        for (ExpansionHubMotor motor : motors) {
//            // TODO: decide whether or not to use the built-in velocity PID
//            // if you keep it, then don't tune kStatic or kA
//            // otherwise, comment out the following line
//            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        }
//
//        // TODO: reverse any motors using DcMotor.setDirection()
//
//        // TODO: set the tuned coefficients from DriveVelocityPIDTuner if using RUN_USING_ENCODER
//         setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDCoefficients(0,0,0));


    }

    public TrajectoryBuilder trajectoryBuilder() {
        return new TrajectoryBuilder(getPoseEstimate(), constraints);
    }

    public void followTrajectory(Trajectory trajectory) {
        follower.followTrajectory(trajectory);
    }

    public void updateFollower() {
        follower.update(getPoseEstimate());
    }

    public void update() {
        updatePoseEstimate();
        updateFollower();
    }

    public boolean isFollowingTrajectory() {
        return follower.isFollowing();
    }

    public Pose2d getFollowingError() {
        return follower.getLastError();
    }

//    @Override
    public PIDCoefficients getPIDCoefficients(DcMotor.RunMode runMode) {
        PIDFCoefficients coefficients = leftMotors.get(0).getPIDFCoefficients(runMode);
        return new PIDCoefficients(coefficients.p, coefficients.i, coefficients.d);
    }

//    @Override
    public void setPIDCoefficients(DcMotor.RunMode runMode, PIDCoefficients coefficients) {
        for (ExpansionHubMotor motor : motors) {
            motor.setPIDFCoefficients(runMode, new PIDFCoefficients(
                    coefficients.kP, coefficients.kI, coefficients.kD, 1
            ));
        }
    }

    @NotNull
    @Override
    public List<Double> getWheelPositions() {
        double leftSum = 0, rightSum = 0;
        RevBulkData bulkData = driveHub.getBulkInputData();

        if (bulkData == null) {
            return Arrays.asList(0.0, 0.0);
        }

        for (DcMotorEx leftMotor : leftMotors) {
            leftSum += DriveConstants.encoderTicksToInches(bulkData.getMotorCurrentPosition(leftMotor));
        }
        for (DcMotorEx rightMotor : rightMotors) {
            rightSum += DriveConstants.encoderTicksToInches(bulkData.getMotorCurrentPosition(rightMotor));
        }
        return Arrays.asList(leftSum / leftMotors.size(), rightSum / rightMotors.size());
//        return Arrays.asList(leftSum, rightSum);
    }

    @Override
    public void setMotorPowers(double v, double v1) {
        for (ExpansionHubMotor leftMotor : leftMotors) {
            leftMotor.setPower(v);
        }
        for (ExpansionHubMotor rightMotor : rightMotors) {
            rightMotor.setPower(v1);
        }
    }

    @Override
    public double getExternalHeading() {
        return revIMU.getAngularOrientation().firstAngle;
    }




    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        double sp = 0;

        RevExtensions2.init();
        driveHub = hwMap.get(ExpansionHubEx.class, "driveHub");

//        sv1 = hwMap.servo.get("servo1");

        // Define and Initialize Motors
        fl   = hwMap.get(ExpansionHubMotor.class, "fl");
        fr  = hwMap.get(ExpansionHubMotor.class, "fr");
        bl   = hwMap.get(ExpansionHubMotor.class, "bl");
        br  = hwMap.get(ExpansionHubMotor.class, "br");

        arm   = hwMap.get(DcMotorEx.class, "arm");
        lift   = hwMap.get(ExpansionHubMotor.class, "lift");
        cascade   = hwMap.get(DcMotorEx.class, "cascade");

        intake = hwMap.crservo.get("intake");
        intake2 = hwMap.crservo.get("intake2");

        intake_motor = hwMap.dcMotor.get("intake_motor");

        intakeDoor = hwMap.servo.get("intakeDoor");
        hangRelease = hwMap.servo.get("hangRelease");
        markerDeployer = hwMap.servo.get("markerDeployer");
        hookRelease = hwMap.servo.get("hookRelease");

//        revIMU = hwMap.get(BNO055IMU.class, "revIMU");
//        revIMU.initialize(new BNO055IMU.Parameters());
        revIMU = LynxOptimizedI2cFactory.createLynxEmbeddedImu(driveHub.getStandardModule(), 0);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        revIMU.initialize(parameters);

        fl.setPower(sp);
        fr.setPower(sp);
        bl.setPower(sp);
        br.setPower(sp);
//        arm.setPower(sp);
        lift.setPower(sp);
//        cascade.setPower(sp);

        arm.setVelocity(0, AngleUnit.DEGREES);
        cascade.setVelocity(0, AngleUnit.DEGREES);

        cascade.setDirection(DcMotorSimple.Direction.REVERSE);

        intake.setPower(0.1120);
        intake2.setPower(0.1120);

        intakeDoor.setPosition(0);
        hangRelease.setPosition(0.2); // latch hang on start
        markerDeployer.setPosition(0.85);
        hookRelease.setPosition(0.6);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        cascade.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        cascade.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        intake_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);
//        fl.setDirection(DcMotorSimple.Direction.REVERSE);
//        bl.setDirection(DcMotorSimple.Direction.REVERSE);



        constraints = new TankConstraints(DriveConstants.BASE_CONSTRAINTS, DriveConstants.TRACK_WIDTH);
        follower = new TankPIDVAFollower(this, DISPLACEMENT_PID, CROSS_TRACK_PID,
                DriveConstants.kV, DriveConstants.kA, DriveConstants.kStatic);
//        follower = new GVFFollower();
////                (this, DISPLACEMENT_PID, CROSS_TRACK_PID,
////                DriveConstants.kV, DriveConstants.kA, DriveConstants.kStatic);

        motors = Arrays.asList(fl, bl, br, fr);
        leftMotors = Arrays.asList(fl, bl);
        rightMotors = Arrays.asList(fr, br);

        for (ExpansionHubMotor motor : motors) {
            // TODO: decide whether or not to use the built-in velocity PID
            // if you keep it, then don't tune kStatic or kA
            // otherwise, comment out the following line
//            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // TODO: reverse any motors using DcMotor.setDirection()

        // TODO: set the tuned coefficients from DriveVelocityPIDTuner if using RUN_USING_ENCODER
        setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, STANDARD_PID);

    }
//
//    public void turnServoContinuous(double speed){
//        sv1.setPosition(speed);
//    }
//    public void stopServoContinuous(){
//        sv1.setPosition(0);
//    }

    public void resetEncoders()
    {
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        cascade.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        cascade.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void resetDriveEncoders()
    {
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
//    public void encoderDrive(double flspeed, double frspeed, double blspeed, double brspeed, int fltarget, int frtarget, int bltarget, int brtarget) {
//        int newFLTarget;
//        int newFRTarget;
//        int newBLTarget;
//        int newBRTarget;
//
//        // Determine new target position, and pass to motor controller
//        newFLTarget = fl.getCurrentPosition()+fltarget;
//        newFRTarget = fr.getCurrentPosition()+frtarget;
//        newBLTarget = bl.getCurrentPosition()+bltarget;
//        newBRTarget = br.getCurrentPosition()+brtarget;
//
//        resetEncoders();
//
//        fl.setPower(flspeed);
//        fr.setPower(frspeed);
//        bl.setPower(blspeed);
//        br.setPower(brspeed);
//
//        // keep looping while we are still active, and there is time left, and both motors are running.
//        while(Math.abs(fl.getCurrentPosition())<Math.abs(fltarget) || Math.abs(fr.getCurrentPosition())<Math.abs(frtarget) || Math.abs(bl.getCurrentPosition())<Math.abs(bltarget) || Math.abs(br.getCurrentPosition())<Math.abs(brtarget))
//        {
//            if(!(Math.abs(fl.getCurrentPosition())<Math.abs(fltarget)))
//            {
//                fl.setPower(0);
//            }
//            if(!(Math.abs(fr.getCurrentPosition())<Math.abs(frtarget)))
//            {
//                fr.setPower(0);
//            }
//            if(!(Math.abs(bl.getCurrentPosition())<Math.abs(bltarget)))
//            {
//                bl.setPower(0);
//            }
//            if(!(Math.abs(br.getCurrentPosition())<Math.abs(brtarget)))
//            {
//                br.setPower(0);
//            }
//        }
//
//        fl.setPower(0);
//        fr.setPower(0);
//        bl.setPower(0);
//        br.setPower(0);
//    }


    public void driveLimitlessTeleop(double left, double right) {
        fl.setVelocity(-left*360*3 *4/3, AngleUnit.DEGREES);
        fr.setVelocity(-right*360*3 *4/3, AngleUnit.DEGREES);
        bl.setVelocity(-left*360*3 *4/3, AngleUnit.DEGREES);
        br.setVelocity(-right*360*3 *4/3, AngleUnit.DEGREES);
    }
    public void driveLimitless(double left, double right) {
        fl.setPower(left);
        fr.setPower(right);
        bl.setPower(left);
        br.setPower(right);
    }

    public void allStop()
    {
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }

    public int getHeading(){
        return -(int)Math.floor(revIMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle);
    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     * @throws InterruptedException
     */
    public void waitForTick(long periodMs) throws InterruptedException {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }



}
