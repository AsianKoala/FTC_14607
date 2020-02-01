package org.firstinspires.ftc.teamcode.Auto.neils_corner;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.HelperClasses.ppProject.company.Range;
import org.jetbrains.annotations.NotNull;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevBulkData;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.teamcode.HelperClasses.GLOBALS.*;


public class BaseAuto extends BaseOpMode {

    private Point startingPosition;
    protected void setStartingPosition(Point startingPosition) { this.startingPosition = startingPosition; }

    private double startingHeading;
    protected void setStartingHeading(double startingHeading) { this.startingHeading = startingHeading; }



    protected OpenCvCamera phoneCam;



    @Override
    public void runOpMode() {
        super.runOpMode(); // linky boi


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();//open camera
        phoneCam.setPipeline(new StageSwitchingPipeline());//different stages
        phoneCam.startStreaming(rows, cols, OpenCvCameraRotation.SIDEWAYS_LEFT);//display on RC
    }




    /**
     * sensor stuff
     */

    protected RevBulkData masterBulkData() { return masterHub.getBulkInputData(); }
    protected RevBulkData slaveBulkData() { return slaveHub.getBulkInputData(); }


    protected int getMotorBulkDataPosition(ExpansionHubEx hub, DcMotor motor) {
        if(hub.equals(masterHub)) {
            return masterBulkData().getMotorCurrentPosition(motor);
        }

        return slaveBulkData().getMotorCurrentPosition(motor);
    }


    protected double getWorldX() {
        return startingPosition.x + encoderTicksToInches(getMotorBulkDataPosition(masterHub, horizontalModule));
    }

    protected double getWorldY() {
        return startingPosition.y + encoderTicksToInches(getMotorBulkDataPosition(masterHub, verticalModule));
    }

    /**
     * pretty sure this wont work lol
     */
    protected double getHeadingRad180(){
        return (imu.getAngularOrientation().firstAngle)+startingHeading;
    }




    /**
     * movement stuff
     */


    /**
     * basic apply dimensional power method
     * @param xPower horizontal power
     * @param yPower vertical
     * @param turnPower turning
     */
    public void driveMecanum(double xPower,double yPower,double turnPower) {

        double rawFL = -yPower+turnPower-xPower*1.5;
        double rawBL = yPower+turnPower- xPower*1.5;
        double rawBR = -yPower-turnPower-xPower*1.5;
        double rawFR = yPower-turnPower-xPower*1.5;


        double scaleAmt = 1;
        double biggestPower = Math.abs(rawFL);

        if(Math.abs(rawBL) > (biggestPower)) { biggestPower = rawBL; }
        if(Math.abs(rawFR) > (biggestPower)) { biggestPower = rawFR; }
        if(Math.abs(rawBR) > (biggestPower)) { biggestPower = rawBR; }
        if(biggestPower > 1.0) {
            scaleAmt = 1.0/biggestPower;
        }

        rawFL *= scaleAmt;
        rawFR *= scaleAmt;
        rawBL *= scaleAmt;
        rawBR *= scaleAmt;


        leftFront.setPower(rawFL);
        rightFront.setPower(rawFR);
        leftRear.setPower(rawBL);
        rightRear.setPower(rawBR);
    }



    protected void lineFor(double inches, double maxSpeed, double timeout) {
        double distanceTraveled = 0;
        while(opModeIsActive() && Math.abs(inches - distanceTraveled) < 0.25) {

        }
    }

    //i dont think we can use this because we dont have 3 wheel odometry
//    protected void goToPosition(double targetX, double targetY, double pointAngle, double movementSpeed, double turnSpeed, double slowDownTurnRad, boolean stop) {
//
//        double distanceToTarget = Math.hypot(targetX - getWorldX(), targetY - getWorldY());
//        double startHeading = getHeadingRad180();
//
//        double movement_x;
//        double movement_y;
//        double movement_turn;
//
//
//        while(distanceToTarget > 0.5 && Math.abs(pointAngle - getHeadingRad180()) < Math.toRadians(2)) {
//
//            double angleToTarget = Math.atan2(targetY - getWorldY(), targetX - getWorldX());
//
//            double relativeAngleToTarget = AngleWrap(angleToTarget - (getHeadingRad180() - Math.toRadians(90)));
//
//
//            double deltaX = targetX - getWorldX();
//            double deltaY = targetY - getWorldY();
//            double absDeltaX = Math.abs(deltaX);
//            double absDeltaY = Math.abs(deltaY);
//
//
//            double xMovementComponent = (deltaX / (absDeltaX + absDeltaY));
//            double yMovementComponent = (deltaY / (absDeltaX + absDeltaY));
//
//            /**
//             * deccel over 12 in
//             */
//            if(stop) {
//                xMovementComponent = deltaX / 12;
//                yMovementComponent = deltaY / 12;
//            }
//
//            movement_x = Range.clip(xMovementComponent, -movementSpeed, movementSpeed);
//            movement_y = Range.clip(yMovementComponent, -movementSpeed, movementSpeed);
//
//
//            /**
//             * now deal with all the turning correction
//             */
//            double relativeTurnAngle = relativeAngleToTarget - Math.toRadians(180) + pointAngle;
//
//            movement_turn = Range.clip(relativeTurnAngle / slowDownTurnRad, -turnSpeed, turnSpeed);
//
//
//            if(distanceToTarget < 12) {
//                movement_turn = 0;
//            }
//
//
//
//            // if something fucked up
//            if(startHeading - getHeadingRad180() > Math.toRadians(120)) {
//                return;
//            }
//
//
//            driveMecanum(movement_x, movement_y, movement_turn);
//            distanceToTarget = Math.hypot(targetX - getWorldX(), targetY - getWorldY());
//        }
//    }


//    protected void verticalMovement(double inches, double maxSpeed, double prefAngle, double timeout, boolean stop) {
//        double startingY = getWorldY();
//        double startingX = getWorldX();
//
//        while(getMotorBulkDataPosition(masterHub, verticalModule) - startingY < inches) {
//
//            double distanceToTarget =
//        }
//    }




    /**
     * cv starts here
     */

    private static int valMid = -1;
    private static int valLeft = -1;
    private static int valRight = -1;

    private static float rectHeight = .6f/8f;
    private static float rectWidth = 1.5f/8f;


    private static float[] midPos = {4.2f/8f, 3.4f/8f};//0 = col, 1 = row
    private static float[] leftPos = {2.5f/8f, 3.4f/8f}; //2.7f/8f
    private static float[] rightPos = {5.9f/8f, 3.4f/8f};
//
//    private static float[] midPos = {4.3f/8f, 2.0f/8f};//0 = col, 1 = row
//    private static float[] leftPos = {2.3f/8f, 2.0f/8f}; //2.7f/8f
//    private static float[] rightPos = {6.3f/8f, 2.0f/8f};
    //moves all rectangles right or left by amount. units are in ratio to monitor

    private final int rows = 640;
    private final int cols = 480;


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
