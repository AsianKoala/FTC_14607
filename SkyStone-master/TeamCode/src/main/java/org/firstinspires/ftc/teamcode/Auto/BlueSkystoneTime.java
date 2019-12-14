package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import java.util.ArrayList;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Auto.roadrunner.util.AxesSigns;
import org.firstinspires.ftc.teamcode.Auto.roadrunner.util.BNO055IMUUtil;
import org.firstinspires.ftc.teamcode.HelperClasses.GLOBALS;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.revextensions2.ExpansionHubMotor;

import java.util.List;

import static org.firstinspires.ftc.teamcode.HelperClasses.GLOBALS.ourSkystonePosition;
import static org.firstinspires.ftc.teamcode.HelperClasses.GLOBALS.psuedoHomer;

import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@Autonomous(name = "Blue Skystone TIME", group = "Firefly")
public class BlueSkystoneTime extends LinearOpMode {

    public static final String TAG = "Vuforia Navigation Sample";

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
    public final static double gripperHome = 0.82;
    public final static double gripperGrip = 0.19;

    public  static double P = 15;
    public  static double I = 0.005;
    public  static double D = 6.2045;

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


        capstone.setPosition(0.9);
        flipper.setPosition(0.22);
        rotater.setPosition(rotaterHome);
        gripper.setPosition(gripperHome);

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
        }

        double startHeading = imu.getAngularOrientation().firstAngle;
        double heading = getHeading(startHeading);

        phoneCam.closeCameraDevice();

        leftSlide.setTargetPosition(-10);//changed to pseudo home
        rightSlide.setTargetPosition(-10);
        leftSlide.setPower(0.75);
        rightSlide.setPower(0.75);

        if(ourSkystonePosition == GLOBALS.SKYSTONE_POSITION.MIDDLE) {
            xPower = -0.3;
            yPower = 0;
            zPower = 0;
            driveMecanum(xPower, yPower, zPower);
            sleep(200);
            
            xPower = 0;
            yPower = 0;
            zPower = 0;
            driveMecanum(xPower, yPower, zPower);
            sleep(100);

            leftIntake.setPower(-0.7);
            rightIntake.setPower(-0.7);
            sleep(250);

            xPower = 0;
            yPower = 0.5;
            zPower = 0;
            driveMecanum(xPower, yPower, zPower);
            sleep(2000);

            xPower = 0;
            yPower = 0;
            zPower = 0;
            driveMecanum(xPower, yPower, zPower);
            sleep(250);

            leftIntake.setPower(-1);
            rightIntake.setPower(-1);
            sleep(250);

            xPower = 0;
            yPower = -0.5;
            zPower = 0;
            driveMecanum(xPower, yPower, zPower);
            sleep(1500);


            flipper.setPosition(flipperHome);

            xPower = 0;
            yPower = 0;
            zPower = 0;
            driveMecanum(xPower, yPower, zPower);
            sleep(500);
            gripper.setPosition(gripperGrip);


            ///////////////
            xPower = 0;
            yPower = 0;
            zPower = -0.5;
            driveMecanum(xPower, yPower, zPower);
            while(opModeIsActive() && getHeading(startHeading)>-Math.PI/2){
                telemetry.addData("imu heading", getHeading(startHeading));
                telemetry.update();
            }
            xPower = 0;
            yPower = 0;
            zPower = 0;
            driveMecanum(xPower, yPower, zPower);

            xPower = 0;
            yPower = 0;
            zPower = 0.25;
            driveMecanum(xPower, yPower, zPower);
            while(opModeIsActive() && getHeading(startHeading)<-Math.PI/2){

            }
            xPower = 0;
            yPower = 0;
            zPower = -0.25;
            driveMecanum(xPower, yPower, zPower);
            sleep(100);
            xPower = 0;
            yPower = 0;
            zPower = 0;
            driveMecanum(xPower, yPower, zPower);
            sleep(100);
            //////////////////////

            leftIntake.setPower(0);
            rightIntake.setPower(0);

            xPower = 0;
            yPower = 0.5;
            zPower = 0;
            driveMecanum(xPower, yPower, zPower);
            sleep(2750);

            xPower = 0;
            yPower = 0;
            zPower = 0;
            driveMecanum(xPower, yPower, zPower);
            sleep(250);



            ///////////////
            xPower = 0;
            yPower = 0;
            zPower = 0.5;
            driveMecanum(xPower, yPower, zPower);
            while(opModeIsActive() && getHeading(startHeading)<0){
                telemetry.addData("imu heading", getHeading(startHeading));
                telemetry.update();
            }
            xPower = 0;
            yPower = 0;
            zPower = 0;
            driveMecanum(xPower, yPower, zPower);

            xPower = 0;
            yPower = 0;
            zPower = -0.25;
            driveMecanum(xPower, yPower, zPower);
            while(opModeIsActive() && getHeading(startHeading)>0){

            }
            xPower = 0;
            yPower = 0;
            zPower = 0;
            driveMecanum(xPower, yPower, zPower);
            //////////////////////

            xPower = 0;
            yPower = 0.3;
            zPower = 0;
            driveMecanum(xPower, yPower, zPower);
            sleep(1000);

            xPower = 0;
            yPower = 0;
            zPower = 0;
            driveMecanum(xPower, yPower, zPower);
            sleep(500);

            grabFoundation();
            sleep(1100);

            xPower = 0;
            yPower = -0.65;
            zPower = 0;
            driveMecanum(xPower, yPower, zPower);
            sleep(1200);

            xPower = 0;
            yPower = -0.2;
            zPower = 0;
            driveMecanum(xPower, yPower, zPower);
            sleep(600);

            xPower = 0;
            yPower = 0;
            zPower = 0;
            driveMecanum(xPower, yPower, zPower);
            sleep(250);

            ungrabFoundation();

            xPower = 0.5;
            yPower = 0;
            zPower = 0;
            driveMecanum(xPower, yPower, zPower);
            sleep(1850);

            xPower = 0;
            yPower = 0.5;
            zPower = 0;
            driveMecanum(xPower, yPower, zPower);
            sleep(700);

            xPower = -0.5;
            yPower = 0;
            zPower = 0;
            driveMecanum(xPower, yPower, zPower);
            sleep(1200);

            xPower = 0.5;
            yPower = 0;
            zPower = 0;
            driveMecanum(xPower, yPower, zPower);
            sleep(300);

            xPower = 0;
            yPower = 0;
            zPower = 0;
            driveMecanum(xPower, yPower, zPower);

            flipper.setPosition(flipperBetween);
            sleep(600);

            leftSlide.setTargetPosition(-400);//changed to pseudo home
            rightSlide.setTargetPosition(-400);
            leftSlide.setPower(0.75);
            rightSlide.setPower(0.75);

            ///////////////
            xPower = 0;
            yPower = 0;
            zPower = 0.75;
            driveMecanum(xPower, yPower, zPower);
            while(opModeIsActive() && getHeading(startHeading)<-5*Math.PI/4 && getHeading(startHeading)>-Math.PI/4){
                telemetry.addData("imu heading", getHeading(startHeading));
                telemetry.update();
            }
            xPower = 0;
            yPower = 0;
            zPower = 0;
            driveMecanum(xPower, yPower, zPower);

            xPower = 0;
            yPower = 0;
            zPower = -0.6;
            driveMecanum(xPower, yPower, zPower);
            while(opModeIsActive() && getHeading(startHeading)>-5*Math.PI/4){

            }
            xPower = 0;
            yPower = 0;
            zPower = 0;
            driveMecanum(xPower, yPower, zPower);
            //////////////////////

            rotater.setPosition(rotaterHome);
            flipper.setPosition(0.7);
            sleep(1000);
            gripper.setPosition(gripperHome);
            sleep(1500);
            flipper.setPosition(flipperBetween);
            sleep(1500);
            leftSlide.setTargetPosition(-20);//changed to pseudo home
            rightSlide.setTargetPosition(-20);
            leftSlide.setPower(0.75);
            rightSlide.setPower(0.75);
            sleep(1000);

            xPower = 0;
            yPower = 0.5;
            zPower = 0;
            driveMecanum(xPower, yPower, zPower);
            sleep(200);

            xPower = 0;
            yPower = 0;
            zPower = 0;
            driveMecanum(xPower, yPower, zPower);



        } else if(ourSkystonePosition == GLOBALS.SKYSTONE_POSITION.LEFT) {

        } else if(ourSkystonePosition == GLOBALS.SKYSTONE_POSITION.RIGHT) {

        } else {
            // oops
        }

    }

    public double getHeading(double startHeading){
        return -(imu.getAngularOrientation().firstAngle-startHeading);
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