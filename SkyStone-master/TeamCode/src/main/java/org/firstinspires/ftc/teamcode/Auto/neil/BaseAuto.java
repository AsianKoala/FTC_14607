package org.firstinspires.ftc.teamcode.Auto.neil;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import net.frogbots.ftcopmodetunercommon.opmode.TunableLinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.teamcode.Auto.roadrunner.util.AxesSigns;
import org.firstinspires.ftc.teamcode.Auto.roadrunner.util.BNO055IMUUtil;
import org.firstinspires.ftc.teamcode.HelperClasses.GLOBALS;
import org.firstinspires.ftc.teamcode.HelperClasses.ppProject.company.Range;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.teamcode.HelperClasses.GLOBALS.*;


public class BaseAuto extends TunableLinearOpMode {

    private ExpansionHubMotor leftFront, leftRear, rightFront, rightRear;
    private ExpansionHubMotor leftIntake, rightIntake;
    private ExpansionHubMotor horizontalModule, verticalModule;
    private ExpansionHubMotor leftSlide, rightSlide;

    private Servo clawRotater, clawGripper, clawFlipper, leftHook, rightHook, capstoneDeployer, parkingDeployer, horizontalExtend, frontGripper, backGripper;




    private BNO055IMU imu;
    private ExpansionHubEx masterHub, slaveHub;
    private OpenCvCamera phoneCam;



    private ArrayList<ExpansionHubMotor> driveMotors = new ArrayList<ExpansionHubMotor>() {{
        add(leftFront);
        add(leftRear);
        add(rightFront);
        add(rightRear);
    }};


    public final static double P = 15;
    public final static double I = 0.005;
    public final static double D = 6.2045;





    @Override
    public void runOpMode() {
        leftFront = hardwareMap.get(ExpansionHubMotor.class, "leftFront");
        leftRear = hardwareMap.get(ExpansionHubMotor.class, "leftRear");
        rightFront = hardwareMap.get(ExpansionHubMotor.class, "rightFront");
        rightRear = hardwareMap.get(ExpansionHubMotor.class, "rightRear");
        leftIntake = hardwareMap.get(ExpansionHubMotor.class, "leftIntake");
        rightIntake = hardwareMap.get(ExpansionHubMotor.class, "rightIntake");
        leftSlide = hardwareMap.get(ExpansionHubMotor.class, "leftSlide");
        rightSlide = hardwareMap.get(ExpansionHubMotor.class, "rightSlide");


        clawRotater = hardwareMap.get(Servo.class, "clawRotater");
        clawGripper = hardwareMap.get(Servo.class, "clawGripper");
        clawFlipper = hardwareMap.get(Servo.class, "clawFlipper");
        leftHook = hardwareMap.get(Servo.class, "leftHook");
        rightHook = hardwareMap.get(Servo.class, "rightHook");
        capstoneDeployer = hardwareMap.get(Servo.class, "capstoneDeployer");
        parkingDeployer = hardwareMap.get(Servo.class, "parkingDeployer");
        horizontalExtend = hardwareMap.get(Servo.class, "horizontalExtend");
        frontGripper = hardwareMap.get(Servo.class, "frontGripper");
        backGripper = hardwareMap.get(Servo.class, "backGripper");




        horizontalModule = hardwareMap.get(ExpansionHubMotor.class, "horizontalModule");
        verticalModule = hardwareMap.get(ExpansionHubMotor.class, "verticalModule");


        imu = hardwareMap.get(BNO055IMU.class, "imu");
        masterHub = hardwareMap.get(ExpansionHubEx.class, "masterHub");
        slaveHub = hardwareMap.get(ExpansionHubEx.class, "slaveHub");


        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
        leftSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        leftSlide.setTargetPosition(0);
        rightSlide.setTargetPosition(0);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftSlide.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDCoefficients(P,I,D));
        rightSlide.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDCoefficients(P,I,D));



        for(ExpansionHubMotor motor : driveMotors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        horizontalModule.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalModule.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);




        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
        // TODO: BNO055IMUUtil.remapAxes(imu, something something);


        BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN);




        telemetry.addLine("ready");
        telemetry.update();


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();//open camera
        phoneCam.setPipeline(new StageSwitchingPipeline());//different stages
        phoneCam.startStreaming(rows, cols, OpenCvCameraRotation.SIDEWAYS_LEFT);//display on RC




        while(!isStarted() && !isStopRequested()) {
            if(valLeft == 0) {
                ourSkystonePosition = GLOBALS.SKYSTONE_POSITION.LEFT;
                telemetry.addData("skystone: ", "left");
            }

            if(valRight == 0) {
                ourSkystonePosition = GLOBALS.SKYSTONE_POSITION.RIGHT;
                telemetry.addData("skystone: ", "right");
            }

            if(valMid == 0) {
                ourSkystonePosition = GLOBALS.SKYSTONE_POSITION.MIDDLE;
                telemetry.addData("skystone: ", "middle");
            }
            telemetry.addData("heading: ", getHeadingRad180());
            telemetry.addData("im blue da ba dee da ba da: ", "blue");
            telemetry.update();
        }
        
        
        phoneCam.closeCameraDevice();
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



    protected double getScaledVerticalEncoder() {
        return encoderTicksToInches(getMotorBulkDataPosition(masterHub, verticalModule));
    }


    protected double getScaledHorizontalEncoder() {
        return encoderTicksToInches(getMotorBulkDataPosition(masterHub, horizontalModule));
    }




    /**
     * pretty sure this wont work lol
     */
    protected double getHeadingRad180(){
        return -AngleWrap(imu.getAngularOrientation().firstAngle);
    }

    protected double subtractAngle(double a1, double a2) {
        return AngleWrap(a1 - a2);
    }



    protected enum STATUS {
        firstMovement,
        secondMovement,
        thirdMovement,
        fourthMovement,
        fifthMovement
    }

    private static ArrayList<subMethod> subMethods = new ArrayList<>();


    protected abstract static class subMethod {
        STATUS subMethodStatus;

        protected subMethod(STATUS status) {
            this.subMethodStatus = status;
            subMethods.add(this);
        }

        protected abstract void method();
    }


    private static void handleSubMethods(STATUS currStatus) {
        for(subMethod e : subMethods) {
            if(e.subMethodStatus == currStatus) {
                e.method();
            }
        }
    }





    /**
     * movement stuff
     */

    protected void stopDriveMotors() {
        driveMecanum(0,0,0);
    }


    /**
     * @param xPower horizontal power
     * @param yPower vertical
     * @param turnPower turning
     */
    protected void driveMecanum(double xPower,double yPower,double turnPower) {

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





    protected void bestVerticalMovement(double inches, double movementSpeed, double minSpeed, double startUpInches, double slowDownInches, double turnSpeed, double slowDownTurnRad, STATUS methodStatus) {

        double scaledVerticalDistanceTraveled = 0;
        double scaledHorizontalDistanceTraveled = 0;
        double verticalStart = getScaledVerticalEncoder();
        double horizontalStart = getScaledHorizontalEncoder();
        double startHeading = getHeadingRad180();

        double vTarget = verticalStart + inches;


        while(Math.abs(vTarget - getScaledVerticalEncoder()) > 1 || Math.abs(getScaledHorizontalEncoder() - horizontalStart) > 0.5) {

            double x_component;
            double y_component;


            if(Math.abs(scaledVerticalDistanceTraveled) < startUpInches) {
                y_component = scaledVerticalDistanceTraveled / startUpInches + minSpeed;
                if(vTarget < getScaledVerticalEncoder()) {
                    y_component = scaledVerticalDistanceTraveled / startUpInches - minSpeed;
                }
            } else {
                y_component = (inches - scaledVerticalDistanceTraveled) / slowDownInches;
            }


            x_component = -scaledHorizontalDistanceTraveled / slowDownInches;


            x_component = Range.clip(x_component, -movementSpeed, movementSpeed);
            y_component = Range.clip(y_component, -movementSpeed, movementSpeed);



            // now deal with turn correction

            double radToTarget = AngleWrap(startHeading - getHeadingRad180());

            double adjustSpeed = 0.125;
            double turn_component = (radToTarget / slowDownTurnRad) * adjustSpeed;
            turn_component = Range.clip(turn_component, -turnSpeed, turnSpeed);


            driveMecanum(x_component, y_component, turn_component);
            handleSubMethods(methodStatus);


            // update
            scaledVerticalDistanceTraveled = getScaledVerticalEncoder() - verticalStart;
            scaledHorizontalDistanceTraveled = getScaledHorizontalEncoder() - horizontalStart;
        }

        stopDriveMotors();
    }





    protected void bestHorizontalMovement(double inches, double movementSpeed, double minSpeed, double startUpInches, double slowDownInches, double turnSpeed, double slowDownTurnRad, STATUS methodStatus) {

        double scaledVerticalDistanceTraveled = 0;
        double scaledHorizontalDistanceTraveled = 0;
        double verticalStart = getScaledVerticalEncoder();
        double horizontalStart = getScaledHorizontalEncoder();

        double xTarget = horizontalStart + inches;
        double startHeading = getHeadingRad180();


        while(Math.abs(xTarget - getScaledHorizontalEncoder()) > 1 || Math.abs(getScaledVerticalEncoder() - verticalStart) > 0.25) {

            double x_component;
            double y_component;


            if(Math.abs(scaledHorizontalDistanceTraveled) < startUpInches) {
                x_component = scaledHorizontalDistanceTraveled / startUpInches + minSpeed;
                if(xTarget < getScaledHorizontalEncoder()) {
                    x_component = scaledHorizontalDistanceTraveled / startUpInches - minSpeed;
                }
            } else {
                x_component = (inches - scaledHorizontalDistanceTraveled) / slowDownInches;
            }


            y_component = -scaledVerticalDistanceTraveled / slowDownInches;

            x_component = Range.clip(x_component, -movementSpeed, movementSpeed);
            y_component = Range.clip(y_component, -movementSpeed, movementSpeed);




            // now deal with turning

            double radToTarget = AngleWrap(startHeading - getHeadingRad180());

            double turnAdjustSpeed = 0.125;
            double turn_component = (radToTarget / slowDownTurnRad) * turnAdjustSpeed;
            turn_component = Range.clip(turn_component, -turnSpeed, turnSpeed);



            // apply motor powers and update vars

            driveMecanum(x_component, y_component, turn_component);
            handleSubMethods(methodStatus);

            scaledHorizontalDistanceTraveled = getScaledHorizontalEncoder() - horizontalStart;
            scaledVerticalDistanceTraveled = getScaledVerticalEncoder() - verticalStart;
        }

        stopDriveMotors();
    }




    protected void turnTo(double targetAngle, double speed, double slowDownRad) {

        while(Math.abs(AngleWrap(targetAngle - getHeadingRad180())) > Math.toRadians(0.5)) {
            double radToTarget = AngleWrap(targetAngle - getHeadingRad180());

            final double adjustSpeed = 0.25;
            double turnComponent = (radToTarget / slowDownRad) * adjustSpeed;
            turnComponent = Range.clip(turnComponent, -speed, speed);;

            driveMecanum(0,0, turnComponent);
        }

        stopDriveMotors();
    }




    /**
     * other sub system stuff
     */



    protected void setIntakePowers(double leftPower, double rightPower) {
        leftIntake.setPower(Range.clip(leftPower, -1, 1));
        rightIntake.setPower(Range.clip(rightPower, -1, 1));
    }

    protected void turnOnIntake() {
        setIntakePowers(1,1);
    }

    protected void turnOffIntake() {
        setIntakePowers(0,0);
    }

    private int liftTargetPosition = 0;
    private long timeTargetLastSet = 0;
    protected void setRawLiftTargetPosition(int encoderTicks, double power) {
        liftTargetPosition = -encoderTicks;
        leftSlide.setTargetPosition(liftTargetPosition);
        rightSlide.setTargetPosition(liftTargetPosition);
        leftSlide.setPower(power);
        rightSlide.setPower(power);
        timeTargetLastSet = System.currentTimeMillis();
    }
    protected void setLiftTargetPosition(int encoderTicks, double power) {
        liftTargetPosition = (int) Range.clip(-encoderTicks, 0, maxLiftHeightEncoderTicks);
        leftSlide.setTargetPosition(liftTargetPosition);
        rightSlide.setTargetPosition(liftTargetPosition);
        leftSlide.setPower(power);
        rightSlide.setPower(power);
        timeTargetLastSet = System.currentTimeMillis();
    }

    protected int getLiftTargetPosition() { return -liftTargetPosition; }
    protected int getLiftCurrPosition() { return -(leftSlide.getCurrentPosition() + rightSlide.getCurrentPosition()); }
    protected void setLiftHome(double power) { setLiftTargetPosition(0, power); }
    public boolean checkLiftPos(int threshold, int position) {
        return Math.abs(getLiftCurrPosition() - position) < threshold;
    }
    protected void setLiftPositionTolerance(int tolerance) {
        leftSlide.setTargetPositionTolerance(tolerance);
        rightSlide.setTargetPosition(tolerance);
    }
    protected boolean checkLiftStalledTime(long timeout) { return System.currentTimeMillis() - timeTargetLastSet > timeout; }
    protected void stopLift() {
        leftSlide.setPower(0);
        rightSlide.setPower(0);
    }
    protected void setLiftEmergencyDisable() {
        leftSlide.setMotorDisable();
        rightSlide.setMotorDisable();
    }
    protected void setLiftEmergencyEnable() {
        leftSlide.setMotorEnable();
        rightSlide.setMotorEnable();
    }


    protected void setHorizontalExtendHome(){
        horizontalExtend.setPosition(GLOBALS.horizontalExtendHome);
    }
    protected void setHorizontalExtendOut(){
        horizontalExtend.setPosition(GLOBALS.horizontalExtendOut);
    }
    protected void setHorizontalExtendFeed(){
        horizontalExtend.setPosition(GLOBALS.horizontalExtendFeed);
    }
    protected void setHorizontalExtendManual(double position){
        horizontalExtend.setPosition(Range.clip(position, GLOBALS.horizontalExtendHome, GLOBALS.horizontalExtendOut));
    }
    protected double getHorizontalExtendPosition() {
        return horizontalExtend.getPosition();
    }

    // FOUNDATION GRIPPER CONTROL
    protected void setFoundationHookGrip() {
        leftHook.setPosition(GLOBALS.leftHookGrip);
        rightHook.setPosition(GLOBALS.rightHookGrip);
    }
    protected void setFoundationHookHome() {
        leftHook.setPosition(GLOBALS.leftHookHome);
        rightHook.setPosition(GLOBALS.rightHookHome);
    }
    protected boolean getFoundationHookState() {
        return leftHook.getPosition() == GLOBALS.leftHookGrip;
    }

    // CAPSTONE CONTROL
    protected void setCapstoneOut() {
        capstoneDeployer.setPosition(GLOBALS.capstoneDeployerOut);
    }
    protected void setCapstoneHome() {
        capstoneDeployer.setPosition(GLOBALS.capstoneDeployerHome);
    }
    protected double getCapstonePosition() {
        return capstoneDeployer.getPosition();
    }

    // GRIPPERS CONTROL
    protected void setFrontGripperOpen() {
        frontGripper.setPosition(GLOBALS.frontGripperOpen);
    }
    protected void setFrontGripperClosed() {
        frontGripper.setPosition(GLOBALS.frontGripperClosed);
    }
    protected double getFrontGripperPosition() {
        return frontGripper.getPosition();
    }
    protected void setBackGripperOpen() {
        backGripper.setPosition(GLOBALS.backGripperOpen);
    }
    protected void setBackGripperClosed() {
        backGripper.setPosition(GLOBALS.backGripperClosed);
    }
    protected double getBackGripperPosition() {
        return backGripper.getPosition();
    }

    // PARKING DEPLOYER CONTROL
    protected void setParkingDeployerOut() {
        parkingDeployer.setPosition(GLOBALS.parkingDeployerOut);
    }
    protected void setParkingDeployerIn() {
        parkingDeployer.setPosition(GLOBALS.parkingDeployerIn);
    }
    protected void setParkingDeployerPosition(double position) {
        parkingDeployer.setPosition(Range.clip(position, GLOBALS.parkingDeployerOut, GLOBALS.parkingDeployerIn));
    }
    protected double getParkingDeployerPosition() {
        return parkingDeployer.getPosition();
    }





    

//
//
//
//
//
//    protected void goToPosition(double targetX, double targetY, double movementSpeed, double turnSpeed, double slowDownTurnRad, boolean stop) {
//
//        double distanceToTarget = Math.hypot(targetX - getWorldX(), targetY - getWorldY());
//        double startHeading = getHeadingRad180();
//
//        double movement_x = 0;
//        double movement_y = 0;
//        double movement_turn = 0;
//
//
//        while(distanceToTarget > 0.5) {
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
//            double relativeTurnAngle = relativeAngleToTarget - Math.toRadians(180) + startHeading;
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
//
//
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