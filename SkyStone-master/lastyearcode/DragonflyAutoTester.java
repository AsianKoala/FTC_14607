package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "Dragonfly TESTER", group = "Dragonfly")

public class DragonflyAutoTester extends LinearOpMode {
    HardwareDragonfly robot = new HardwareDragonfly();

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private static final String VUFORIA_KEY = "AeCc8pP/////AAABmR47b8z1C0g6laofaiYlml5P0gPtRVgPAQS5Q7s5734f4+PCmqPO3TliZJsnQMsIdzZM5kaAyRjD3xugYYzAgSMyuMvE+mPDUnH8YX6D3Msb8GTtGETdN0sFYKdsoB6i4XXz4K81I8Gj9W5aPwSN5X649dJ4QjtsIvCj5s7aIFZJ8R0EnyoVTk3GaNTcX96ew0BDoUnbg2VqwpTj9QZigizg0b7ZuSQI3o4iZ83llYyINsqPnWoLU49TCk3qFxdXrhu5DBRMVXMIm3tnz9bsgG0+flvJIBJua17xCMevpn2BSdRb1SbyM/buoykJ0XYgz4+i2PBnWZZO4iZ1cNgXKvW8ahLem4fMFs7rx5gBwPJJ\n";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
        robot.resetEncoders();
        telemetry.addData("Say", "Hello Driver");
        updateTelemetry(telemetry);

        robot.lift.setPower(0); // brake lift motor to keep robot hanging in place

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /** Wait for the game to begin */
        telemetry.addData("status", "waiting for start");
        updateTelemetry(telemetry);
        waitForStart();

        int globalStartHeading = robot.getHeading();

        long timeOfStart = System.currentTimeMillis();

        int goldState = 1; // 0 = left, 1 = center, 2 = right
        if (opModeIsActive()) {
            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }
            boolean twoObjectsFound = false;
            while(opModeIsActive() && !twoObjectsFound){ // 5 second timeout in case phone does not recognize minerals
            if (tfod != null) {
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                List<Recognition> updatedRecognitionsCropped = new ArrayList<Recognition>(updatedRecognitions);

                //start of sketchy
                for(Recognition x : updatedRecognitions){
                    telemetry.addData(x.getLabel(), x.getTop());
                    if(x.getTop()<600){ //above 60 px0 from the top
                        updatedRecognitionsCropped.remove(x);
                    }
                }
                updatedRecognitions = updatedRecognitionsCropped;
                //end of sketchy

                if (updatedRecognitions != null) {
                    if (updatedRecognitions.size() == 2) {
                        twoObjectsFound = true;
                        telemetry.addData("Yes two objects found: ", updatedRecognitions.size());
                        boolean silverCenter = false;
                        boolean silverRight = false;
                        // telemetry.addData("recognitions", updatedRecognitions.toString());
                        if(updatedRecognitions.get(0).getLabel().equals(LABEL_SILVER_MINERAL)){
                            if(updatedRecognitions.get(0).getLeft()<updatedRecognitions.get(1).getLeft()){
                                silverCenter = true;
                            }else{
                                silverRight = true;
                            }
                        }
                        if(updatedRecognitions.get(1).getLabel().equals(LABEL_SILVER_MINERAL)){
                            if(updatedRecognitions.get(1).getLeft()>updatedRecognitions.get(0).getLeft()) {
                                silverRight = true;
                            }else{
                                silverCenter = true;
                            }
                        }

                        if(silverCenter && silverRight){ // GOLD IS LEFT
                            telemetry.addData("GOLD: ", "LEFT");
                            goldState = 0;
                        }else if(!silverCenter && silverRight){ // GOLD IS LEFT
                            telemetry.addData("GOLD: ", "CENTER");
                            goldState = 1;
                        }else if(silverCenter && !silverRight){ // GOLD IS LEFT
                            telemetry.addData("GOLD: ", "RIGHT");
                            goldState = 2;
                        }
                    }else{
                        telemetry.addData("No two objects found: ", updatedRecognitions.size());
                    }
                    telemetry.update();
                }
            }}

        }
        if (tfod != null) {
            tfod.shutdown();
        }

        while (opModeIsActive()){ // wait until end of opmode
            telemetry.addData("Autonomous completed... ", 0);
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    final double WHEEL_CIRCUMFERENCE_INCHES = Math.PI * 4;
    public double encoderValToInches(int val){
//        double rotations = ((double)val)/537.6;
//        double inches_travelled = rotations*WHEEL_CIRCUMFERENCE_INCHES;
//        return inches_travelled;
        return ((double)val)/(1000/24);
    }

    public void moveForward(double power, double inches){
        robot.resetDriveEncoders();
        //sleep(200);
        int startLeftEncoderPos = robot.fl.getCurrentPosition();
        //int startRightEncoderPos = robot.fr.getCurrentPosition();
        robot.fl.setPower(-power);
        robot.fr.setPower(-power);
        robot.bl.setPower(-power);
        robot.br.setPower(-power);
        while(opModeIsActive() && encoderValToInches(robot.fl.getCurrentPosition()-startLeftEncoderPos) < inches) {
        }
//        while(opModeIsActive() && encoderValToInches(robot.fl.getCurrentPosition()-startLeftEncoderPos) < inches || encoderValToInches(robot.fr.getCurrentPosition()-startRightEncoderPos) < inches) {
//            if (encoderValToInches(robot.fl.getCurrentPosition() - startLeftEncoderPos) >= inches) {
//                robot.fl.setPower(0);
//                robot.bl.setPower(0);
//            }
//            if (encoderValToInches(robot.fr.getCurrentPosition() - startLeftEncoderPos) >= inches) {
//                robot.fr.setPower(0);
//                robot.br.setPower(0);
//            }
//        }
        robot.allStop();
    }

    public void turnEncoders(double power, double val){
        //sleep(200);
        int startLeftEncoderPos = robot.fl.getCurrentPosition();
        //int startRightEncoderPos = robot.fr.getCurrentPosition();
        robot.fl.setPower(-power);
        robot.fr.setPower(power);
        robot.bl.setPower(-power);
        robot.br.setPower(power);
        while(opModeIsActive() && Math.abs(robot.fl.getCurrentPosition()-startLeftEncoderPos) < val) {
        }
//        while(opModeIsActive() && encoderValToInches(robot.fl.getCurrentPosition()-startLeftEncoderPos) < inches || encoderValToInches(robot.fr.getCurrentPosition()-startRightEncoderPos) < inches) {
//            if (encoderValToInches(robot.fl.getCurrentPosition() - startLeftEncoderPos) >= inches) {
//                robot.fl.setPower(0);
//                robot.bl.setPower(0);
//            }
//            if (encoderValToInches(robot.fr.getCurrentPosition() - startLeftEncoderPos) >= inches) {
//                robot.fr.setPower(0);
//                robot.br.setPower(0);
//            }
//        }
        robot.allStop();
    }

    public void moveBackwards(double power, double inches){
        int startLeftEncoderPos = robot.fl.getCurrentPosition();
        int startRightEncoderPos = robot.fr.getCurrentPosition();
        robot.fl.setPower(power);
        robot.fr.setPower(power);
        robot.bl.setPower(power);
        robot.br.setPower(power);
        while(opModeIsActive() && encoderValToInches(robot.fl.getCurrentPosition()-startLeftEncoderPos) > -inches || encoderValToInches(robot.fr.getCurrentPosition()-startRightEncoderPos) > -inches) {
            if (encoderValToInches(robot.fl.getCurrentPosition() - startLeftEncoderPos) <= -inches) {
                robot.fl.setPower(0);
                robot.bl.setPower(0);
            }
            if (encoderValToInches(robot.fr.getCurrentPosition() - startLeftEncoderPos) <= -inches) {
                robot.fr.setPower(0);
                robot.br.setPower(0);
            }
        }
        robot.allStop();
    }

    public void unlockHang(){
        robot.lift.setPower(-0.5);
        robot.hangRelease.setPosition(0);
        sleep(500);
        robot.lift.setPower(0);
        sleep(1000); // wait for locker to move completely out of way before drop
    }

    public void lowerHangPowerless(int timeMillis){
        robot.lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        sleep(timeMillis);
        robot.lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void detachHang(){
        while(opModeIsActive() && robot.lift.getCurrentPosition() > -4684){
            robot.lift.setPower(0.5);
        }
        robot.lift.setPower(0);
        robot.hookRelease.setPosition(0.2); //0.13 correct
        sleep(750);
        robot.hookRelease.setPosition(0.6);
        sleep(750);
        robot.hookRelease.setPosition(0.2);
        sleep(750);
        robot.hookRelease.setPosition(0.6);
        sleep(750);
        robot.hookRelease.setPosition(0.2);
        sleep(1000);
    }

    public void resetHang(){
        robot.hookRelease.setPosition(0.6);
        while(opModeIsActive() && robot.lift.getCurrentPosition() < -100){
            robot.lift.setPower(-0.3);
        }
        robot.lift.setPower(0);
    }

    public void turn(double power, double degrees){ // positive power and positive degrees for right turn
        int startHeading = robot.getHeading();
        int headingDiff = robot.getHeading()-startHeading;
        while(Math.abs(headingDiff)<Math.abs(degrees)){
            headingDiff = robot.getHeading()-startHeading;
            robot.driveLimitless(power, -power);
        }
        robot.allStop();
    }

    public void turn(double power, double degrees, int startHeading){ // positive power and positive degrees for right turn
        int headingDiff = robot.getHeading()-startHeading;
        while(Math.abs(headingDiff)<Math.abs(degrees)){
            headingDiff = robot.getHeading()-startHeading;
            robot.driveLimitless(power, -power);
        }
        robot.allStop();
    }

    public void dropMarker(){
        robot.markerDeployer.setPosition(0);
        sleep(1000);
        robot.markerDeployer.setPosition(0.1);
        sleep(100);
        robot.markerDeployer.setPosition(0);
        sleep(100);
        robot.markerDeployer.setPosition(0.1);
        sleep(100);
        robot.markerDeployer.setPosition(0);
        sleep(100);
        robot.markerDeployer.setPosition(0.85);
        sleep(1000);
    }
}