package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.Auto.roadrunner.util.AxesSigns;
import org.firstinspires.ftc.teamcode.Auto.roadrunner.util.BNO055IMUUtil;
import org.firstinspires.ftc.teamcode.Teleop.FireFlyRobot;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvPipeline;


import java.util.ArrayList;
import java.util.List;


@Autonomous(name = "Blue Odo Auto", group = "Firefly")
public class BlueFinalOdoAuto extends LinearOpMode {


    FireFlyRobot robot = new FireFlyRobot();

    public static final String TAG = "Vuforia Navigation Sample";

    private double startHeading;
    private double lastAngleFound;

    OpenGLMatrix lastLocation = null;

    VuforiaLocalizer vuforia;

    //0 means skystone, 1 means yellow stone
    //-1 for debug, but we can keep it like this because if it works, it should change to either 0 or 255
    private static int valMid = -1;
    private static int valLeft = -1;
    private static int valRight = -1;

    private static float rectHeight = .6f/8f;
    private static float rectWidth = 1.5f/8f;


    private static float[] midPos = {4.65f/8f, 2.2f/8f};//0 = col, 1 = row
    private static float[] leftPos = {2.95f/8f, 2.2f/8f}; //2.7f/8f
    private static float[] rightPos = {6.35f/8f, 2.2f/8f};
//
//    private static float[] midPos = {4.3f/8f, 2.0f/8f};//0 = col, 1 = row
//    private static float[] leftPos = {2.3f/8f, 2.0f/8f}; //2.7f/8f
//    private static float[] rightPos = {6.3f/8f, 2.0f/8f};
    //moves all rectangles right or left by amount. units are in ratio to monitor

    private final int rows = 640;
    private final int cols = 480;

    OpenCvCamera phoneCam;

    private BNO055IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        robot.init(hardwareMap);
        robot.initPositions();

        robot.setDriveMotorsVelocityMode();

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);


        BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN);


//
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
//        phoneCam.openCameraDevice();//open camera
//        phoneCam.setPipeline(new StageSwitchingPipeline());//different stages
//        phoneCam.startStreaming(rows, cols, OpenCvCameraRotation.SIDEWAYS_LEFT);//display on RC
        //width, height
        //width = height in this case, because camera is in portrait mode.
//
        while(!isStarted() && !isStopRequested()) {
            telemetry.addData("Ready.", 0);
            telemetry.addData("heading: ", getHeadingRaw180(startHeading));
            telemetry.update();
        }

        startHeading = getHeadingRaw180(0);
        lastAngleFound = startHeading;
//        phoneCam.closeCameraDevice();

        int state = 1;

        double flpower = 0;
        double frpower = 0;
        double blpower = 0;
        double brpower = 0;

        xOdoStartVal = xOdoVal();
        yOdoStartVal = yOdoVal();

        long stateStartTime = System.currentTimeMillis();

        while(opModeIsActive()) {

            frpower = 0;
            brpower = 0;
            flpower = 0;
            blpower = 0;

            if(getHeadingRaw180(startHeading) > 3) {
                frpower += 0.1;
                brpower += 0.1;
                flpower -= 0.1;
                blpower -= 0.1;
            } else if(getHeadingRaw180(startHeading) < -3) {
                frpower -= 0.1;
                brpower -= 0.1;
                flpower += 0.1;
                blpower += 0.1;
            }

            if(state == 1) {
                // move to y = -26

                if(yOdoVal() > -22) { // -26
                    frpower -= 0.3;
                    brpower -= 0.3;
                    flpower -= 0.3;
                    blpower -= 0.3;
                } else {
                    state = 2;
                    stateStartTime = System.currentTimeMillis();
                }

            }

            if(state == 2) {
                // move to x = 9

                if(xOdoVal() < 15) {
                    frpower += 0.4; // 0.3
                    brpower -= 0.4;
                    flpower += 0.4;
                    blpower -= 0.4;
                } else {
                    state = 3;
                    stateStartTime = System.currentTimeMillis();
                }
            }

            if(state == 3) {
                //move to y = -30

                if(yOdoVal() > -32) {
                    frpower = -0.2;
                    brpower = -0.2;
                    flpower = -0.2;
                    blpower = -0.2;
                } else {
                    state = 4;
                    stateStartTime = System.currentTimeMillis();
                }
            }

            if(state == 4) {
                // stop
                if(System.currentTimeMillis() - stateStartTime < 1000){
                    robot.setFoundationHookGrip();
                }else {
                    state = 5;
                    stateStartTime = System.currentTimeMillis();
                }
            }

            if(state == 5) {
                //move to y = -15

                if(yOdoVal() < -15) {
                    frpower += 0.3;
                    brpower += 0.3;
                    flpower += 0.3;
                    blpower += 0.3;
                } else {
                    state = 6;
                    stateStartTime = System.currentTimeMillis();
                }
            }

            if(state == 6) {
                //move to y = -15

                if(getHeadingRaw180(startHeading) > -95) { //90
                    frpower += 0.4;
                    brpower += 0.4;
                    flpower -= 0.1;
                    blpower -= 0.1;
                } else {
                    state = 7;
                    stateStartTime = System.currentTimeMillis();
                }
            }

            if(state == 7) {
                //move to y = -15

                if(System.currentTimeMillis() - stateStartTime < 2500) { // 1000
                    frpower -= 0.3;
                    brpower -= 0.3;
                    flpower -= 0.3;
                    blpower -= 0.3;
                } else {
                    state = 8;
                    stateStartTime = System.currentTimeMillis();
                    robot.setFoundationHookHome();
                }
            }

            if(state == 8) {
                // move to x = -20 // -16

                if(System.currentTimeMillis() - stateStartTime < 2000) {
                    frpower += 0.3;
                    brpower -= 0.3;
                    flpower += 0.3;
                    blpower -= 0.3;
                } else {
                    state = 9;
                    stateStartTime = System.currentTimeMillis();
                    // RESET ODO ENCODERS
                    xOdoStartVal = xOdoVal() + xOdoStartVal;
                    yOdoStartVal = yOdoVal() + yOdoStartVal;
                }
            }

            if(state == 9) {
                // move to y = -26

                if(yOdoVal() < 36) {
                    frpower += 0.4;
                    brpower += 0.4;
                    flpower += 0.4;
                    blpower += 0.4;
                } else {
                    state = 10;
                    stateStartTime = System.currentTimeMillis();
                }

            }

            if(state == 10) {
                flpower = 0;
                frpower = 0;
                blpower = 0;
                brpower = 0;
                telemetry.addData("done", 0);
            }



            robot.setFLPower(flpower);
            robot.setFRPower(frpower);
            robot.setBLPower(blpower);
            robot.setBRPower(brpower);


            telemetry.addData("yOdoVal: ", yOdoVal());
            telemetry.addData("xOdoVal: ", xOdoVal());
            telemetry.addData("heading: ", getHeadingRaw180(startHeading));
            telemetry.update();
        }

        robot.setDriveStop();

    }

    public void resetOdoEncoders() {
        robot.leftIntake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightIntake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    double yOdoStartVal;
    public double yOdoVal() {
        return (-robot.leftIntake.getCurrentPosition() * 0.0230097118 / 25.4) - yOdoStartVal;
    }

    double xOdoStartVal;
    public double xOdoVal() {
        return (-robot.rightIntake.getCurrentPosition() * 0.0230097118 / 25.4) - xOdoStartVal;
    }

    
    private double wraparoundCorrection = 0.0;
    public double getHeadingRaw180(double startHeading){
        double raw = Math.toDegrees(imu.getAngularOrientation().firstAngle)+startHeading;
        raw = -raw;

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