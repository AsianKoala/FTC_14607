package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.Auto.roadrunner.util.AxesSigns;
import org.firstinspires.ftc.teamcode.Auto.roadrunner.util.BNO055IMUUtil;
import org.firstinspires.ftc.teamcode.HelperClasses.GLOBALS;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.revextensions2.ExpansionHubMotor;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.teamcode.HelperClasses.GLOBALS.ourSkystonePosition;


@Autonomous(name = "REAL Red Skystone", group = "Firefly")
public class RedSkystoneEncoder extends LinearOpMode {

    public static final String TAG = "Vuforia Navigation Sample";

    private double startHeading;
    private double lastAngleFound;

    OpenGLMatrix lastLocation = null;

    VuforiaLocalizer vuforia;

    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftRear;
    private DcMotor rightRear;
    private DcMotor leftIntake;
    private DcMotor rightIntake;
    private ExpansionHubMotor leftSlide;
    private ExpansionHubMotor rightSlide;
    private Servo flipper, gripper, rotater, leftSlam, rightSlam, capstone;
    private BNO055IMU imu;

    private ArrayList<DcMotor> driveMotors = new ArrayList<>();

    private double oldSlideLeft = 0;
    private double oldSlideRight = 0;
    private double newSlideLeft = 0;
    private double newSlideRight = 0;

    //0 means skystone, 1 means yellow stone
    //-1 for debug, but we can keep it like this because if it works, it should change to either 0 or 255
    private static int valMid = -1;
    private static int valLeft = -1;
    private static int valRight = -1;

    private static float rectHeight = .6f/8f;
    private static float rectWidth = 1.5f/8f;


    private static float[] midPos = {4.3f/8f, 2.7f/8f};//0 = col, 1 = row
    private static float[] leftPos = {2.3f/8f, 2.7f/8f};
    private static float[] rightPos = {6.3f/8f, 2.7f/8f};
    //moves all rectangles right or left by amount. units are in ratio to monitor

    private final int rows = 640;
    private final int cols = 480;

    OpenCvCamera phoneCam;

    public  final static double flipperHome =  0.15;
    public final  static double flipperOut = 0.8513;
    public  final static double flipperBetween = 0.3;
    public   static double rotaterHome = 0.279;
    public  static double rotaterOut = 0.95;
    public final static double gripperHome = 0.19;
    public final static double gripperGrip = 0.82;

    public  static double P = 15;
    public  static double I = 0.005;
    public  static double D = 6.2045;

    public void resetEncoders() {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setTargetPosition(0);
        leftRear.setTargetPosition(0);
        rightFront.setTargetPosition(0);
        rightRear.setTargetPosition(0);
//        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void runOpMode() throws InterruptedException {


        imu = hardwareMap.get(BNO055IMU.class, "imu");

        leftFront = hardwareMap.get(DcMotor.class, "FL");
        leftRear = hardwareMap.get(DcMotor.class, "BL");
        rightRear = hardwareMap.get(DcMotor.class, "FR");
        rightFront = hardwareMap.get(DcMotor.class, "BR");
        leftIntake = hardwareMap.get(DcMotor.class, "leftIntake");
        rightIntake = hardwareMap.get(DcMotor.class, "rightIntake");
        leftSlide = hardwareMap.get(ExpansionHubMotor.class, "leftSlide");
        rightSlide = hardwareMap.get(ExpansionHubMotor.class, "rightSlide");

        gripper = hardwareMap.get(Servo.class, "gripper");
        flipper = hardwareMap.get(Servo.class, "flipper");
        rotater = hardwareMap.get(Servo.class, "rotater");
        leftSlam = hardwareMap.get(Servo.class, "leftSlam");
        rightSlam = hardwareMap.get(Servo.class, "rightSlam");
        capstone = hardwareMap.get(Servo.class, "capstone");



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

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        resetEncoders();



        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);


        BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN);



        double yPower = 0;
        double xPower = 0;
        double zPower = 0;

        telemetry.addData("Ready.", 0);
        telemetry.update();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();//open camera
        phoneCam.setPipeline(new StageSwitchingPipeline());//different stages
        phoneCam.startStreaming(rows, cols, OpenCvCameraRotation.SIDEWAYS_LEFT);//display on RC
        //width, height
        //width = height in this case, because camera is in portrait mode.


//        capstone.setPosition(0.9);
        flipper.setPosition(0.22);
        rotater.setPosition(GLOBALS.rotaterHome);
        gripper.setPosition(GLOBALS.gripperHome);

        startHeading = getHeadingRaw180(0);
        lastAngleFound = startHeading;

        while(!isStarted() && !isStopRequested()) {
            if(valLeft == 0) {
                ourSkystonePosition = GLOBALS.SKYSTONE_POSITION.LEFT;
            }

            if(valRight == 0) {
                ourSkystonePosition = GLOBALS.SKYSTONE_POSITION.RIGHT;
            }
    
            if(valMid == 0) {
                ourSkystonePosition = GLOBALS.SKYSTONE_POSITION.MIDDLE;
            }
            telemetry.addData("heading: ", getHeadingRaw180(startHeading));
            telemetry.update();
        }


//        double heading = getHeadingRaw180(startHeading);

        phoneCam.closeCameraDevice();

        grabFoundation();

        leftSlide.setTargetPosition(-10);//changed to pseudo home
        rightSlide.setTargetPosition(-10);
        leftSlide.setPower(0.75);
        rightSlide.setPower(0.75);

        if(ourSkystonePosition == GLOBALS.SKYSTONE_POSITION.MIDDLE) {
            middleStone();
        } else if(ourSkystonePosition == GLOBALS.SKYSTONE_POSITION.LEFT) {
            leftStone();
        } else if(ourSkystonePosition == GLOBALS.SKYSTONE_POSITION.RIGHT) {
            rightStone();
        } else {
            // oops
        }

    }

    public void leftStone(){

    }

    public void middleStone(){

        leftIntake.setPower(-.7);
        rightIntake.setPower(-.7);

        driveEncodersForwardIMU(2086, 2086, 2086, 2086, .4, .4, .4, .4, 0.25, 5, 0, true); // 0.4

        rotateToSquare(0, 0.2, 1);

        leftIntake.setPower(-1);
        rightIntake.setPower(-1);

        ungrabFoundation();
        
        
        // TODO: COMPENSATE, ORIGINALLY 1200

        driveEncodersForwardIMU(1550, 1550, 1550, 1550, -.4, -.4, -.4, -.4, 0.25, 5, 0, false);

        leftIntake.setPower(-0.7);
        rightIntake.setPower(-0.7);

        rotateToSquare(0, 0.2, 1);

        leftIntake.setPower(-1);
        rightIntake.setPower(-1);


//        gripper.setPosition(GLOBALS.gripperGrip);
//        gripper.setPosition(GLOBALS.gripperHome);

        ///////////////


//
//        leftIntake.setPower(-0.1);
//        rightIntake.setPower(-0.1);



        driveEncodersStrafeIMU(1550+3000, 1550+3000, 1550-3000, 1550-3000, 1, 1, -1, -1, 0.5, 6, 0, true);

        flipper.setPosition(GLOBALS.flipperHome);

//        leftIntake.setPower(-1);
//        rightIntake.setPower(-1);

        rotateToSquare(0, 0.2, 1);

        // DROP BLOCK HERE

//        flipper.setPosition(GLOBALS.flipperHome);
//        gripper.setPosition(GLOBALS.gripperGrip);
//        sleep(250);

        /////////////////////////



        sleep(200);
        gripper.setPosition(GLOBALS.gripperGrip);
        sleep(500);
        flipper.setPosition(GLOBALS.flipperBetween);
        sleep(250);

        leftIntake.setPower(0);
        rightIntake.setPower(0);

        leftIntake.setPower(0.3);
        rightIntake.setPower(0.3);

        leftSlide.setTargetPosition(-400);
        rightSlide.setTargetPosition(-400);
        leftSlide.setPower(0.9);
        rightSlide.setPower(0.9);
        sleep(250);

        leftIntake.setPower(0);
        rightIntake.setPower(0);

        rotater.setPosition(GLOBALS.rotaterOut);
        sleep(700); // 500
        flipper.setPosition(GLOBALS.flipperOut);
        sleep(500);
        gripper.setPosition(GLOBALS.gripperHome);

        /////////////////////////////////



//        sleep(500);

        // DONE DROPPING FIRST STONE

//        gripper.setPosition(GLOBALS.gripperHome);
//        flipper.setPosition(GLOBALS.flipperBetween);
//        sleep(500);
//        rotater.setPosition(GLOBALS.rotaterHome);
//        sleep(500);
//
//        leftSlide.setTargetPosition(-10);
//        rightSlide.setTargetPosition(-10);
//        leftSlide.setPower(0.5);
//        rightSlide.setPower(0.5);
//        sleep(500);


        rotateToSquareResetLift(-90, 0.5, 5);
        rotateToSquare(-90, 0.2, 1);
        rotateToSquare(-90, 0.15, 1);

        resetEncoders();

        driveEncodersForwardIMU(3350, 3350, 3350, 3350, 1, 1, 1, 1, 0.4, 4.5, -90, true);

        rotateToSquare(-90, 0.2, 1);
        
        //700 strafe

        driveEncodersStrafeIMU(760+3350, 760+3350, -760+3350, -760+3350, .6, .6, -.6, -.6, 0.4, 6, -90, true);

        leftIntake.setPower(-0.7);
        rightIntake.setPower(-0.7);

        rotateToSquare(-90, 0.2, 1);

        driveEncodersForwardIMU(300+760+3350, 300+760+3350, 300-760+3350, 300-760+3350, .9, .9, .9, .9, 0.3, 5, -90, true);

        leftIntake.setPower(-1);
        rightIntake.setPower(-1);

        rotateToSquare(-90, 0.2, 1);

        resetEncoders();

        flipper.setPosition(GLOBALS.flipperHome);

        // was 700
        driveEncodersStrafeIMU(-750, -750, 750, 750, -.6, -.6, .6, .6, 0.3, 4, -90, false);

        gripper.setPosition(GLOBALS.gripperGrip);

        rotateToSquare(-90, 0.2, 1);
        rotateToSquare(-90, 0.15, 1);



        leftIntake.setPower(0.3);
        rightIntake.setPower(0.3);

        resetEncoders();

        // -3300
        driveEncodersForwardIMU(-3300, -3300, -3300, -3300, -1, -1, -1, -1, 0.8, 5, -90, false);

        leftIntake.setPower(0);
        rightIntake.setPower(0);

        rotateToSquare(0, 0.6, 4);
        rotateToSquare(0, 0.3, 1);

        resetEncoders();

//        driveEncodersForwardIMU(300, 300, 300, 300, .5, .5, .5, .5, 0.45, 2, 0, true);

        leftFront.setPower(0.6);
        leftRear.setPower(0.6);
        rightFront.setPower(0.6);
        rightRear.setPower(0.6);

        grabFoundation();
        sleep(500);

        leftFront.setPower(-0.6);
        leftRear.setPower(-0.6);
        rightFront.setPower(-0.6);
        rightRear.setPower(-0.6);

        sleep(500);

        rotateToSquare(100, 0.65, 4);

//        leftFront.setPower(-0.1);
//        leftRear.setPower(-0.1);
//        rightFront.setPower(-0.8);
//        rightRear.setPower(-0.8);
//
//        sleep(1450); // 1200

        leftFront.setPower(0.55);
        leftRear.setPower(0.55);
        rightFront.setPower(0.55);
        rightRear.setPower(0.55);

        ungrabFoundation();

        sleep(2000);

        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);


        resetEncoders();

        driveEncodersForwardIMU(-2100, -2100, -2100, -2100, -.6, -.6, -.6, -.6, 0.5, 5, 90, false);




    }

    public void rightStone(){

    }


    public void driveEncodersForwardIMU(int fl_target, int fr_target, int bl_target, int br_target, double flpower, double frpower, double blpower, double brpower, double minPower, double timeoutSec, double targetHeading, boolean positive) {
        int fl_start = leftFront.getCurrentPosition();
        int fr_start = rightFront.getCurrentPosition();
        int bl_start = leftRear.getCurrentPosition();
        int br_start = rightRear.getCurrentPosition();

        double min_drive_motor_power = minPower;

        boolean fl_reached = false;
        boolean fr_reached = false;
        boolean bl_reached = false;
        boolean br_reached = false;

        double real_fl_power = 0;
        double real_fr_power = 0;
        double real_bl_power = 0;
        double real_br_power = 0;

        double startTime = System.currentTimeMillis();

        while(!(fl_reached && fr_reached && bl_reached && br_reached) && opModeIsActive() && (System.currentTimeMillis()-startTime)/1000 < timeoutSec) {

            double powerLeft = Math.max(min_drive_motor_power + Math.abs(flpower-min_drive_motor_power) * Math.min(Math.abs(leftFront.getCurrentPosition()-fl_start), Math.abs(fl_target-leftFront.getCurrentPosition())) / Math.abs(fl_target-fl_start) * 2, min_drive_motor_power);
            double powerRight = Math.max(min_drive_motor_power + Math.abs(flpower-min_drive_motor_power) * Math.min(Math.abs(leftFront.getCurrentPosition()-fl_start), Math.abs(fl_target-leftFront.getCurrentPosition())) / Math.abs(fl_target-fl_start) * 2, min_drive_motor_power);

            if(Math.abs(leftFront.getCurrentPosition()-fl_start) >= Math.abs(fl_target-fl_start)) {
                fl_reached = true;
                fr_reached = true;
                bl_reached = true;
                br_reached = true;
                real_fl_power = 0;
            }
            
            if(getHeadingRaw180(startHeading) > targetHeading) {
                if(positive) {
                    powerRight += Math.abs(targetHeading-getHeadingRaw180(startHeading)) / 30 * powerRight;
                } else {
                    powerLeft += Math.abs(targetHeading-getHeadingRaw180(startHeading)) / 30 * powerRight;
                }

            } else {
                if(positive) {
                    powerLeft += Math.abs(targetHeading-getHeadingRaw180(startHeading)) / 30 * powerRight;
                } else {
                    powerRight += Math.abs(targetHeading-getHeadingRaw180(startHeading)) / 30 * powerRight;
                }
            }
            
            real_fl_power = Math.signum(flpower) * powerLeft;
            real_fr_power = Math.signum(frpower) * powerRight;//Math.max(Math.abs(frpower) * Math.min(Math.abs(rightFront.getCurrentPosition()-fr_start), Math.abs(fr_target-rightFront.getCurrentPosition())) / Math.abs(fr_target-fr_start) * 2, min_drive_motor_power));
            real_bl_power = (Math.signum(blpower) * powerLeft);//Math.max(Math.abs(blpower) * Math.min(Math.abs(leftRear.getCurrentPosition()-bl_start), Math.abs(bl_target-leftRear.getCurrentPosition())) / Math.abs(bl_target-bl_start) * 2, min_drive_motor_power));
            real_br_power = (Math.signum(brpower) * powerRight);//Math.max(Math.abs(brpower) * Math.min(Math.abs(rightRear.getCurrentPosition()-br_start), Math.abs(br_target-rightRear.getCurrentPosition())) / Math.abs(br_target-br_start) * 2, min_drive_motor_power));
            leftFront.setPower(real_fl_power);
            leftRear.setPower(real_bl_power);
            rightFront.setPower(real_fr_power);
            rightRear.setPower(real_br_power);
        }

        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
    }

    public void driveEncodersStrafeIMU(int fl_target, int fr_target, int bl_target, int br_target, double flpower, double frpower, double blpower, double brpower, double minPower, double timeoutSec, double targetHeading, boolean positive) {
        int fl_start = leftFront.getCurrentPosition();
        int fr_start = rightFront.getCurrentPosition();
        int bl_start = leftRear.getCurrentPosition();
        int br_start = rightRear.getCurrentPosition();

        double min_drive_motor_power = minPower;

        boolean fl_reached = false;
        boolean fr_reached = false;
        boolean bl_reached = false;
        boolean br_reached = false;

        double real_fl_power = 0;
        double real_fr_power = 0;
        double real_bl_power = 0;
        double real_br_power = 0;

        double startTime = System.currentTimeMillis();

        while(!(fl_reached && fr_reached && bl_reached && br_reached) && opModeIsActive() && (System.currentTimeMillis()-startTime)/1000 < timeoutSec) {

            double powerFront = Math.max(min_drive_motor_power + Math.abs(flpower-min_drive_motor_power) * Math.min(Math.abs(leftFront.getCurrentPosition()-fl_start), Math.abs(fl_target-leftFront.getCurrentPosition())) / Math.abs(fl_target-fl_start) * 2, min_drive_motor_power);
            double powerBack = Math.max(min_drive_motor_power + Math.abs(flpower-min_drive_motor_power) * Math.min(Math.abs(leftFront.getCurrentPosition()-fl_start), Math.abs(fl_target-leftFront.getCurrentPosition())) / Math.abs(fl_target-fl_start) * 2, min_drive_motor_power);

            if(Math.abs(leftFront.getCurrentPosition()-fl_start) >= Math.abs(fl_target-fl_start)) {
                fl_reached = true;
                fr_reached = true;
                bl_reached = true;
                br_reached = true;
                real_fl_power = 0;
            }

            if(getHeadingRaw180(startHeading) > targetHeading) {
                if(positive) {
                    powerBack += Math.abs(targetHeading-getHeadingRaw180(startHeading)) / 30 * powerBack;
                } else {
                    powerFront += Math.abs(targetHeading-getHeadingRaw180(startHeading)) / 30 * powerFront;
                }
            } else {
                if(positive) {
                    powerFront += Math.abs(targetHeading-getHeadingRaw180(startHeading)) / 30 * powerFront;

                } else {
                    powerBack += Math.abs(targetHeading-getHeadingRaw180(startHeading)) / 30 * powerBack;
                }
            }

            real_fl_power = Math.signum(flpower) * powerFront;
            real_fr_power = Math.signum(frpower) * powerBack;//Math.max(Math.abs(frpower) * Math.min(Math.abs(rightFront.getCurrentPosition()-fr_start), Math.abs(fr_target-rightFront.getCurrentPosition())) / Math.abs(fr_target-fr_start) * 2, min_drive_motor_power));
            real_bl_power = (Math.signum(blpower) * powerBack);//Math.max(Math.abs(blpower) * Math.min(Math.abs(leftRear.getCurrentPosition()-bl_start), Math.abs(bl_target-leftRear.getCurrentPosition())) / Math.abs(bl_target-bl_start) * 2, min_drive_motor_power));
            real_br_power = (Math.signum(brpower) * powerFront);//Math.max(Math.abs(brpower) * Math.min(Math.abs(rightRear.getCurrentPosition()-br_start), Math.abs(br_target-rightRear.getCurrentPosition())) / Math.abs(br_target-br_start) * 2, min_drive_motor_power));
            leftFront.setPower(real_fl_power);
            leftRear.setPower(real_bl_power);
            rightFront.setPower(real_fr_power);
            rightRear.setPower(real_br_power);
        }

        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
    }

    public void driveEncodersStrafe(int fl_target, int fr_target, int bl_target, int br_target, double flpower, double frpower, double blpower, double brpower, double minPower, double timeoutSec) {
        int fl_start = leftFront.getCurrentPosition();
        int fr_start = rightFront.getCurrentPosition();
        int bl_start = leftRear.getCurrentPosition();
        int br_start = rightRear.getCurrentPosition();

        double min_drive_motor_power = minPower;

        boolean fl_reached = false;
        boolean fr_reached = false;
        boolean bl_reached = false;
        boolean br_reached = false;

        double real_fl_power = 0;
        double real_fr_power = 0;
        double real_bl_power = 0;
        double real_br_power = 0;

        double startTime = System.currentTimeMillis();

        while(!(fl_reached && fr_reached && bl_reached && br_reached) && opModeIsActive() && (System.currentTimeMillis()-startTime)/1000 < timeoutSec) {

            double power = Math.max(min_drive_motor_power + Math.abs(flpower-min_drive_motor_power) * Math.min(Math.abs(leftFront.getCurrentPosition()-fl_start), Math.abs(fl_target-leftFront.getCurrentPosition())) / Math.abs(fl_target-fl_start) * 2, min_drive_motor_power);
            if(Math.abs(leftFront.getCurrentPosition()-fl_start) >= Math.abs(fl_target-fl_start)) {
                fl_reached = true;
                real_fl_power = 0;
            } else {
                real_fl_power = Math.signum(flpower) * power;
            }
            if(Math.abs(rightFront.getCurrentPosition()-fr_start) >= Math.abs(fr_target-fr_start)) {
                fr_reached = true;
                real_fr_power = 0;
            } else {
                real_fr_power = Math.signum(frpower) * power;//Math.max(Math.abs(frpower) * Math.min(Math.abs(rightFront.getCurrentPosition()-fr_start), Math.abs(fr_target-rightFront.getCurrentPosition())) / Math.abs(fr_target-fr_start) * 2, min_drive_motor_power));
            }
            if(Math.abs(leftRear.getCurrentPosition()-bl_start) >= Math.abs(bl_target-bl_start)) {
                bl_reached = true;
                real_bl_power = 0;
            } else {
                real_bl_power = (Math.signum(blpower) * power);//Math.max(Math.abs(blpower) * Math.min(Math.abs(leftRear.getCurrentPosition()-bl_start), Math.abs(bl_target-leftRear.getCurrentPosition())) / Math.abs(bl_target-bl_start) * 2, min_drive_motor_power));
            }
            if(Math.abs(rightRear.getCurrentPosition()-br_start) >= Math.abs(br_target-br_start)) {
                br_reached = true;
                real_br_power = 0;
            } else {
                real_br_power = (Math.signum(brpower) * power);//Math.max(Math.abs(brpower) * Math.min(Math.abs(rightRear.getCurrentPosition()-br_start), Math.abs(br_target-rightRear.getCurrentPosition())) / Math.abs(br_target-br_start) * 2, min_drive_motor_power));
            }
            leftFront.setPower(real_fl_power);
            leftRear.setPower(real_bl_power);
            rightFront.setPower(real_fr_power);
            rightRear.setPower(real_br_power);
        }

        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
    }
    
    
    private double wraparoundCorrection = 0.0;
    public double getHeadingRaw180(double startHeading){
        double raw = Math.toDegrees(imu.getAngularOrientation().firstAngle)+startHeading;
        raw = -raw;
        
        
//        if(raw > 180) {
//            return -(360-raw);
//        } else {
//            return raw;
//        }

        if(Math.abs(lastAngleFound-raw) > 30 && Math.signum(lastAngleFound) == -Math.signum(raw)) {
            if(Math.signum(raw) == -1) { // wrapped around from positive to negative
                wraparoundCorrection += 360;
            } else { // wrapped around from negative to positive
                wraparoundCorrection -= 360;
            }
        }
        
        lastAngleFound = raw;

        raw += wraparoundCorrection;
        return raw;
    }
    
    public void rotateToSquare(double targetHeading, double power, double timeoutSec){
        double currentHeading;
        currentHeading = getHeadingRaw180(startHeading);
        double startTime = System.currentTimeMillis();
        while(opModeIsActive() && currentHeading > targetHeading && (System.currentTimeMillis()-startTime)/1000 < timeoutSec) {
            currentHeading = getHeadingRaw180(startHeading);
            leftFront.setPower(-power);
            leftRear.setPower(-power);
            rightFront.setPower(power);
            rightRear.setPower(power);
            telemetry.addData("heading: ", currentHeading);
            telemetry.update();
        }
        while(opModeIsActive() && currentHeading < targetHeading && (System.currentTimeMillis()-startTime)/1000 < timeoutSec){
            currentHeading = getHeadingRaw180(startHeading);
            leftFront.setPower(power);
            leftRear.setPower(power);
            rightFront.setPower(-power);
            rightRear.setPower(-power);
            telemetry.addData("heading: ", currentHeading);
            telemetry.update();
        }
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
    }

    public void rotateToSquareResetLift(double targetHeading, double power, double timeoutSec){
        double currentHeading;
        currentHeading = getHeadingRaw180(startHeading);
        double startTime = System.currentTimeMillis();



        while(opModeIsActive() && currentHeading > targetHeading && (System.currentTimeMillis()-startTime)/1000 < timeoutSec) {
            currentHeading = getHeadingRaw180(startHeading);
            leftFront.setPower(-power);
            leftRear.setPower(-power);
            rightFront.setPower(power);
            rightRear.setPower(power);
            telemetry.addData("heading: ", currentHeading);
            telemetry.update();

            if((System.currentTimeMillis()-startTime) > 500) {
                gripper.setPosition(GLOBALS.gripperHome);
                flipper.setPosition(GLOBALS.flipperBetween);
            }

            if((System.currentTimeMillis()-startTime) > 1000) {
                rotater.setPosition(GLOBALS.rotaterHome);
            }

            if((System.currentTimeMillis()-startTime) > 1250) {
                leftSlide.setTargetPosition(-10);
                rightSlide.setTargetPosition(-10);
                leftSlide.setPower(0.5);
                rightSlide.setPower(0.5);
            }

        }
        while(opModeIsActive() && currentHeading < targetHeading && (System.currentTimeMillis()-startTime)/1000 < timeoutSec){
            currentHeading = getHeadingRaw180(startHeading);
            leftFront.setPower(power);
            leftRear.setPower(power);
            rightFront.setPower(-power);
            rightRear.setPower(-power);
            telemetry.addData("heading: ", currentHeading);
            telemetry.update();

            if((System.currentTimeMillis()-startTime) > 500) {
                gripper.setPosition(GLOBALS.gripperHome);
                flipper.setPosition(GLOBALS.flipperBetween);
            }

            if((System.currentTimeMillis()-startTime) > 500) {
                rotater.setPosition(GLOBALS.rotaterHome);
            }

            if((System.currentTimeMillis()-startTime) > 1250) {
                leftSlide.setTargetPosition(-10);
                rightSlide.setTargetPosition(-10);
                leftSlide.setPower(0.5);
                rightSlide.setPower(0.5);
            }

        }
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);

        leftSlide.setTargetPosition(-10);
        rightSlide.setTargetPosition(-10);
        leftSlide.setPower(0.5);
        rightSlide.setPower(0.5);
    }

    public void driveMecanum(double xPower,double yPower,double  zPower) {
        yPower = -yPower;
        rightFront.setPower(1 * (((-yPower) + (xPower)) + -zPower));
        leftRear.setPower(1 * (((-yPower) + (-xPower)) + zPower));
        leftFront.setPower(1 * (((-yPower) + (xPower)) + zPower));
        rightRear.setPower(1 * (((-yPower) + (-xPower)) + -zPower));
    }

    public void grabFoundation() {
        leftSlam.setPosition(0.9);
        rightSlam.setPosition(0.1);
    }

    public void ungrabFoundation() {
        leftSlam.setPosition(0.1);
        rightSlam.setPosition(0.9);
    }

    static class StageSwitchingPipeline extends OpenCvPipeline
    {
        Mat yCbCrChan2Mat = new Mat();
        Mat thresholdMat = new Mat();
        Mat all = new Mat();
        List<MatOfPoint> contoursList = new ArrayList<>();

        enum Stage
        {//color difference. greyscale
            detection,//includes outlines
            THRESHOLD,//b&w
            RAW_IMAGE,//displays raw view
        }

        private Stage stageToRenderToViewport = Stage.detection;
        private Stage[] stages = Stage.values();

        @Override
        public void onViewportTapped()
        {
            /*
             * Note that this method is invoked from the UI thread
             * so whatever we do here, we must do quickly.
             */

            int currentStageNum = stageToRenderToViewport.ordinal();

            int nextStageNum = currentStageNum + 1;

            if(nextStageNum >= stages.length)
            {
                nextStageNum = 0;
            }

            stageToRenderToViewport = stages[nextStageNum];
        }

        @Override
        public Mat processFrame(Mat input)
        {
            contoursList.clear();
            /*
             * This pipeline finds the contours of yellow blobs such as the Gold Mineral
             * from the Rover Ruckus game.
             */

            //color diff cb.
            //lower cb = more blue = skystone = white
            //higher cb = less blue = yellow stone = grey
            Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);//converts rgb to ycrcb
            Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);//takes cb difference and stores

            //b&w
            Imgproc.threshold(yCbCrChan2Mat, thresholdMat, 102, 255, Imgproc.THRESH_BINARY_INV);

            //outline/contour
            Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            yCbCrChan2Mat.copyTo(all);//copies mat object
            //Imgproc.drawContours(all, contoursList, -1, new Scalar(255, 0, 0), 3, 8);//draws blue contours


            //get values from frame
            double[] pixMid = thresholdMat.get((int)(input.rows()* midPos[1]), (int)(input.cols()* midPos[0]));//gets value at circle
            valMid = (int)pixMid[0];

            double[] pixLeft = thresholdMat.get((int)(input.rows()* leftPos[1]), (int)(input.cols()* leftPos[0]));//gets value at circle
            valLeft = (int)pixLeft[0];

            double[] pixRight = thresholdMat.get((int)(input.rows()* rightPos[1]), (int)(input.cols()* rightPos[0]));//gets value at circle
            valRight = (int)pixRight[0];

            //create three points
            Point pointMid = new Point((int)(input.cols()* midPos[0]), (int)(input.rows()* midPos[1]));
            Point pointLeft = new Point((int)(input.cols()* leftPos[0]), (int)(input.rows()* leftPos[1]));
            Point pointRight = new Point((int)(input.cols()* rightPos[0]), (int)(input.rows()* rightPos[1]));

            //draw circles on those points
            Imgproc.circle(all, pointMid,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(all, pointLeft,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(all, pointRight,5, new Scalar( 255, 0, 0 ),1 );//draws circle

            //draw 3 rectangles
            Imgproc.rectangle(//1-3
                    all,
                    new Point(
                            input.cols()*(leftPos[0]-rectWidth/2),
                            input.rows()*(leftPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(leftPos[0]+rectWidth/2),
                            input.rows()*(leftPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//3-5
                    all,
                    new Point(
                            input.cols()*(midPos[0]-rectWidth/2),
                            input.rows()*(midPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(midPos[0]+rectWidth/2),
                            input.rows()*(midPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//5-7
                    all,
                    new Point(
                            input.cols()*(rightPos[0]-rectWidth/2),
                            input.rows()*(rightPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(rightPos[0]+rectWidth/2),
                            input.rows()*(rightPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);

            switch (stageToRenderToViewport)
            {
                case THRESHOLD:
                {
                    return thresholdMat;
                }

                case detection:
                {
                    return all;
                }

                case RAW_IMAGE:
                {
                    return input;
                }

                default:
                {
                    return input;
                }
            }
        }

    }

}