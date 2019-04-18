package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;

import java.util.List;

@Autonomous(name = "Dragonfly Marker MP", group = "Dragonfly")

public class DragonflyAutoMarker_MP extends LinearOpMode {
    HardwareDragonflyMP robot = new HardwareDragonflyMP();

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private static final String VUFORIA_KEY = "AeCc8pP/////AAABmR47b8z1C0g6laofaiYlml5P0gPtRVgPAQS5Q7s5734f4+PCmqPO3TliZJsnQMsIdzZM5kaAyRjD3xugYYzAgSMyuMvE+mPDUnH8YX6D3Msb8GTtGETdN0sFYKdsoB6i4XXz4K81I8Gj9W5aPwSN5X649dJ4QjtsIvCj5s7aIFZJ8R0EnyoVTk3GaNTcX96ew0BDoUnbg2VqwpTj9QZigizg0b7ZuSQI3o4iZ83llYyINsqPnWoLU49TCk3qFxdXrhu5DBRMVXMIm3tnz9bsgG0+flvJIBJua17xCMevpn2BSdRb1SbyM/buoykJ0XYgz4+i2PBnWZZO4iZ1cNgXKvW8ahLem4fMFs7rx5gBwPJJ\n";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
        robot.resetEncoders();
        telemetry.addData("Say", "Hello Driver");
        updateTelemetry(telemetry);

        robot.lift.setPower(0); // brake lift motor to keep robot hanging in place

        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        telemetry.addData("status", "waiting for start");
        updateTelemetry(telemetry);

        Trajectory T_1 = robot.trajectoryBuilder()
                .forward(robot.FORWARD_MOVE_MARKER_VAL_EXTRA)
                .waitFor(0.5)
                .build();

        Trajectory T_2 = robot.trajectoryBuilder()
                .back(robot.FORWARD_MOVE_MARKER_VAL_EXTRA-robot.FORWARD_MOVE_SAMPLING_VAL)
                .waitFor(0.5)
                .build();

        Trajectory T_3 = robot.trajectoryBuilder()
                .back(robot.FORWARD_MOVE_SAMPLING_VAL)
                .waitFor(0.5)
                .build();

        Trajectory T_4 = robot.trajectoryBuilder()
                .forward(robot.DRIVE_DEPOT_PARK_MOVE_1)
                .waitFor(0.5)
                .build();

        Trajectory T_5 = robot.trajectoryBuilder()
                .forward(robot.DRIVE_DEPOT_PARK_MOVE_2)
                .waitFor(0.5)
                .build();

        Trajectory T_6 = robot.trajectoryBuilder()
                .forward(robot.DRIVE_DEPOT_PARK_MOVE_3)
                .waitFor(0.5)
                .build();



        while(!isStarted()){
            telemetry.addData("imu val 1: ", -(int)Math.floor(robot.revIMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle));
            telemetry.addData("imu val 2: ", -(int)Math.floor(robot.revIMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle));
            telemetry.addData("imu val 3: ", -(int)Math.floor(robot.revIMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle));
            telemetry.update();
        }
        waitForStart();

        int globalStartHeading = robot.getHeading();

        long timeOfStart = System.currentTimeMillis();

        int goldState = 0; // 0 = left, 1 = center, 2 = right
        if (opModeIsActive()) {
            if (tfod != null) {
                tfod.activate();
            }
            boolean twoObjectsFound = false;
            while(opModeIsActive() && !twoObjectsFound && (System.currentTimeMillis()-timeOfStart)<2500){ // 2.5 second timeout in case phone does not recognize minerals
            if (tfod != null) {
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    if (updatedRecognitions.size() == 2) {
                        twoObjectsFound = true;
                        telemetry.addData("Yes two objects found: ", updatedRecognitions.size());
                        boolean silverCenter = false;
                        boolean silverRight = false;
                        // telemetry.addData("recognitions", updatedRecognitions.toString());
                        if(updatedRecognitions.get(0).getLabel().equals(LABEL_SILVER_MINERAL)){
                            if(updatedRecognitions.get(0).getLeft()<updatedRecognitions.get(1).getLeft()){
                                silverRight = true;
                            }else{
                                silverCenter = true;
                            }
                        }
                        if(updatedRecognitions.get(1).getLabel().equals(LABEL_SILVER_MINERAL)){
                            if(updatedRecognitions.get(1).getLeft()>updatedRecognitions.get(0).getLeft()) {
                                silverCenter = true;
                            }else{
                                silverRight = true;
                            }
                        }

                        if(silverCenter && silverRight){ // GOLD IS LEFT
                            telemetry.addData("GOLD: ", "LEFT");
                            goldState = 0;
                        }else if(!silverCenter && silverRight){ // GOLD IS CENTER
                            telemetry.addData("GOLD: ", "CENTER");
                            goldState = 1;
                        }else if(silverCenter && !silverRight){ // GOLD IS RIGHT
                            telemetry.addData("GOLD: ", "RIGHT");
                            goldState = 2;
                        }
                    }else{
                        telemetry.addData("No two objects found: ", updatedRecognitions.size());
                        if(updatedRecognitions.size() == 1){
                            //more than 265 from left is center, less is right
                            if(updatedRecognitions.get(0).getLabel().equals(LABEL_GOLD_MINERAL)){
                                if(updatedRecognitions.get(0).getLeft()>265){
                                    goldState = 1;
                                    twoObjectsFound = true;
                                }else{
                                    goldState = 2;
                                    twoObjectsFound = true;
                                }
                            }
                            telemetry.addData("gold possible: "+updatedRecognitions.get(0).getLabel(), updatedRecognitions.get(0).getLeft());
                        }else{

                        }
                    }
                    telemetry.update();
                }
            }}

        }
        if (tfod != null) {
            tfod.shutdown();
        }

        // detach and lower from hang
        lowerHangFast();

        //turn out of hang
        while(opModeIsActive() && robot.getHeading()>robot.TURN_OUT_DELATCH_VAL){ robot.driveLimitless(-0.5, 0.5); } //0.3 -0.3
        robot.allStop();

        //reset hang mechanism out of the way of the latch
        resetHangPartial();

        //turn back to face forward

        while(opModeIsActive() && robot.getHeading()<globalStartHeading){ robot.driveLimitless(0.3, -0.3); } //TODO robot.TURN_OUT_RESET_VAL
        robot.allStop();
        sleep(100);
        while(opModeIsActive() && robot.getHeading()>globalStartHeading){ robot.driveLimitless(-0.2, 0.2); } //TODO ?
        robot.allStop();
        sleep(100);
//        third turn correction unnecessary
        while(opModeIsActive() && robot.getHeading()<globalStartHeading){ robot.driveLimitless(0.2, -0.2); } //TODO ?
        robot.allStop();
        sleep(100);

        //lift arm to extend cascade
        robot.arm.setTargetPosition(robot.ARM_MARKER_DEPLOY_VAL);
        robot.arm.setPower(Math.max((Math.abs(robot.arm.getCurrentPosition() - robot.arm.getTargetPosition())) / 100 * (0.2), (0.2)));

        //extend cascade to drop team marker
        if(!robot.cascade.getMode().equals(DcMotor.RunMode.RUN_TO_POSITION)){
            robot.cascade.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        robot.cascade.setTargetPosition(robot.CASCADE_MARKER_EXTEND_VAL);
        robot.cascade.setPower(0.9);

        //drive to marker deployment location
        followTrajectory(T_1);

        while(opModeIsActive()&& robot.arm.isBusy()) {
        }

        while(opModeIsActive()&& robot.cascade.isBusy()){
        }

        //drop team marker by spinning intake out
        robot.intake_motor.setPower(0.85);
        sleep(250); //1000 500

        //retract cascade
        if(!robot.cascade.getMode().equals(DcMotor.RunMode.RUN_TO_POSITION)){
            robot.cascade.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        robot.cascade.setTargetPosition(robot.CASCADE_IN_VAL);
        robot.cascade.setPower(0.9);

        //drive to sampling location
        followTrajectory(T_2);

        while(opModeIsActive()&& robot.cascade.isBusy()){
        }

        //start intake
        robot.intake_motor.setPower(-0.85);

        //switch depending on location of gold mineral
        switch(goldState){
            case 0:

                while(opModeIsActive() && robot.getHeading()>globalStartHeading+robot.TURN_LEFT_GOLD_VAL){ robot.driveLimitless(-0.3, 0.3); } //TODO ?
                robot.allStop();
                sleep(100);

                //extend arm to knock off gold mineral
                robot.arm.setTargetPosition(robot.ARM_LEFT_GOLD_VAL);
                robot.arm.setPower(Math.max((Math.abs(robot.arm.getCurrentPosition() - robot.arm.getTargetPosition())) / 100 * (0.2), (0.2)));

                robot.cascade.setTargetPosition(robot.CASCADE_LEFT_GOLD_EXTEND_VAL);
                robot.cascade.setPower(0.9);

                while(opModeIsActive()&& robot.arm.isBusy()) {
                }
                while(opModeIsActive()&& robot.cascade.isBusy()){
                }

                sleep(750); //wait for gold mineral to be collected //1000

                //retract arm to default position
                robot.arm.setTargetPosition(robot.ARM_TRANSITION_VAL);
                robot.arm.setPower(Math.max((Math.abs(robot.arm.getCurrentPosition() - robot.arm.getTargetPosition())) / 100 * (0.2), (0.2)));

                robot.cascade.setTargetPosition(robot.CASCADE_SCORE_DEFAULT_VAL);
                robot.cascade.setPower(0.9);

                //reset turn position
                while(opModeIsActive() && robot.getHeading()<globalStartHeading){ robot.driveLimitless(0.3, -0.3); } //TODO ?
                robot.allStop();
                sleep(100);

                break;
            case 1:

//                while(opModeIsActive() && robot.getHeading()<globalStartHeading+robot.TURN_CENTER_GOLD_MINADJUST){ robot.driveLimitless(0.3, -0.3); } //TODO ?
//                robot.allStop();
//                sleep(100);

                //extend arm to knock off gold mineral
                robot.arm.setTargetPosition(robot.ARM_CENTER_GOLD_VAL);
                robot.arm.setPower(Math.max((Math.abs(robot.arm.getCurrentPosition() - robot.arm.getTargetPosition())) / 100 * (0.2), (0.2)));

                robot.cascade.setTargetPosition(robot.CASCADE_CENTER_GOLD_EXTEND_VAL);
                robot.cascade.setPower(0.9);

                while(opModeIsActive()&& robot.arm.isBusy()) {
                }
                while(opModeIsActive()&& robot.cascade.isBusy()){
                }

                sleep(750); //wait for gold mineral to be collected

                //retract arm to default position
                robot.arm.setTargetPosition(robot.ARM_TRANSITION_VAL);
                robot.arm.setPower(Math.max((Math.abs(robot.arm.getCurrentPosition() - robot.arm.getTargetPosition())) / 100 * (0.2), (0.2)));

                robot.cascade.setTargetPosition(robot.CASCADE_SCORE_DEFAULT_VAL);
                robot.cascade.setPower(0.9);

                break;
            case 2:

                while(opModeIsActive() && robot.getHeading()<globalStartHeading+robot.TURN_RIGHT_GOLD_VAL){ robot.driveLimitless(0.3, -0.3); } //TODO ?
                robot.allStop();
                sleep(100);

                //extend arm to knock off gold mineral
                robot.arm.setTargetPosition(robot.ARM_RIGHT_GOLD_VAL);
                robot.arm.setPower(Math.max((Math.abs(robot.arm.getCurrentPosition() - robot.arm.getTargetPosition())) / 100 * (0.2), (0.2)));

                robot.cascade.setTargetPosition(robot.CASCADE_RIGHT_GOLD_EXTEND_VAL);
                robot.cascade.setPower(0.9);

                while(opModeIsActive()&& robot.arm.isBusy()) {
                }
                while(opModeIsActive()&& robot.cascade.isBusy()){
                }

                sleep(750); //wait for gold mineral to be collected

                //retract arm to default position
                robot.arm.setTargetPosition(robot.ARM_TRANSITION_VAL);
                robot.arm.setPower(Math.max((Math.abs(robot.arm.getCurrentPosition() - robot.arm.getTargetPosition())) / 100 * (0.2), (0.2)));

                robot.cascade.setTargetPosition(robot.CASCADE_SCORE_DEFAULT_VAL);
                robot.cascade.setPower(0.9);

                //reset turn position
                while(opModeIsActive() && robot.getHeading()<globalStartHeading){ robot.driveLimitless(0.3, -0.3); } //TODO ?
                robot.allStop();
                sleep(100);

                break;
        }

        //correct heading
        while(opModeIsActive() && robot.getHeading()>globalStartHeading){ robot.driveLimitless(-0.3, 0.3); } //TODO ?
        robot.allStop();
        sleep(100);

        while(opModeIsActive() && robot.getHeading()<globalStartHeading){ robot.driveLimitless(0.3, -0.3); } //TODO ?
        robot.allStop();
        sleep(100);

        //drive backwards towards lander to score gold mineral
        followTrajectory(T_3);

        while(opModeIsActive()&& robot.arm.isBusy()) {
        }
        while(opModeIsActive()&& robot.cascade.isBusy()){
        }

        //tilt arm up to score gold mineral
        robot.arm.setTargetPosition(robot.ARM_VERTICAL_VAL);
        robot.arm.setPower(Math.max((Math.abs(robot.arm.getCurrentPosition() - robot.arm.getTargetPosition())) / 100 * (0.2), (0.2)));
        while(opModeIsActive()&& robot.arm.isBusy()) {
        }
        sleep(500);

        //reset arm position
        robot.arm.setTargetPosition(robot.ARM_PARK_VAL);
        robot.arm.setPower(Math.max((Math.abs(robot.arm.getCurrentPosition() - robot.arm.getTargetPosition())) / 100 * (0.2), (0.2)));

        //move out to path to park
        followTrajectory(T_4);

        //turn out to path to park
        while(opModeIsActive() && robot.getHeading()>globalStartHeading+robot.DRIVE_DEPOT_PARK_TURN_1){ robot.driveLimitless(-0.3, 0.3); } //TODO ?
        robot.allStop();
        sleep(100);
        while(opModeIsActive() && robot.getHeading()<globalStartHeading+robot.DRIVE_DEPOT_PARK_TURN_1){ robot.driveLimitless(0.3, -0.3); } //TODO ?
        robot.allStop();
        sleep(100);

        //move out on path to park
        followTrajectory(T_5);

        //turn out on path to park
        while(opModeIsActive() && robot.getHeading()>globalStartHeading+robot.DRIVE_DEPOT_PARK_TURN_2){ robot.driveLimitless(-0.3, 0.3); } //TODO ?
        robot.allStop();
        sleep(100);
        while(opModeIsActive() && robot.getHeading()<globalStartHeading+robot.DRIVE_DEPOT_PARK_TURN_2){ robot.driveLimitless(0.3, -0.3); } //TODO ?
        robot.allStop();
        sleep(100);

        //extend cascade to park
        robot.cascade.setTargetPosition(robot.CASCADE_MARKER_EXTEND_VAL);
        robot.cascade.setPower(0.9);

        //move out on path to park
        followTrajectory(T_6);

        while(opModeIsActive()&& robot.cascade.isBusy()){

        }

        while (opModeIsActive()){ // wait until end of opmode
            telemetry.addData("Autonomous completed... ", 0);
        }


        sleep(500000);
        //FIRST TEMP STOP ––––––––––––––––––––––––––––––––––––––––––––––––––






        //turn out to path to park
        while(opModeIsActive() && robot.getHeading()>robot.TURN_OUT_DRIVE_PARK_VAL_1){ robot.driveLimitless(0.4, -0.4); } //TODO ?
        robot.allStop();
        sleep(100);

        //drive forward to path to park
        moveForward(0.7, robot.FORWARD_MOVE_PARK_VAL_1);

        //extend cascade to park
        robot.cascade.setTargetPosition(robot.CASCADE_MARKER_EXTEND_VAL);
        robot.cascade.setPower(0.9);

        //turn out to path to park
        while(opModeIsActive() && robot.getHeading()>robot.TURN_OUT_DRIVE_PARK_VAL_2){ robot.driveLimitless(0, -0.6); } //TODO ?
        robot.allStop();
        sleep(100);

        //drive forward to path to park
        moveForward(0.7, robot.FORWARD_MOVE_PARK_VAL_2);

        while(opModeIsActive()&& robot.cascade.isBusy()){

        }

        while (opModeIsActive()){ // wait until end of opmode
            telemetry.addData("Autonomous completed... ", 0);
        }
    }

    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "webcam");
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    public void followTrajectory(Trajectory t) {
        robot.followTrajectory(t);
        while (opModeIsActive() && robot.isFollowingTrajectory()) {
            Pose2d currentPose = robot.getPoseEstimate();

            robot.update();
        }
    }

    final double WHEEL_CIRCUMFERENCE_INCHES = Math.PI * 4;
    public double encoderValToInches(int val){
        return ((double)val)/(1000/24);
    }

    public void lowerHangFast(){
        if(!robot.lift.getMode().equals(DcMotor.RunMode.RUN_TO_POSITION)){
            robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        robot.lift.setTargetPosition(robot.LIFT_HOOK_VAL);
        robot.lift.setPower(-1);
        while(robot.lift.isBusy()){
        }
    }

    public void resetHangPartial(){
        if(!robot.lift.getMode().equals(DcMotor.RunMode.RUN_TO_POSITION)){
            robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        robot.lift.setTargetPosition(robot.LIFT_DETATCH_VAL); //clear out of the way
        robot.lift.setPower(-1);
        while(robot.lift.isBusy()){
        }
    }

    public void moveForward(double power, double inches){
        robot.resetDriveEncoders();
        int startLeftEncoderPos = robot.fl.getCurrentPosition();
        int startRightEncoderPos = robot.fr.getCurrentPosition();
        robot.fl.setPower(-power);
        robot.fr.setPower(-power);
        robot.bl.setPower(-power);
        robot.br.setPower(-power);
        while(opModeIsActive() && opModeIsActive() && (Math.abs(encoderValToInches(robot.fl.getCurrentPosition()-startLeftEncoderPos)) < inches || Math.abs(encoderValToInches(robot.fr.getCurrentPosition()-startRightEncoderPos)) < inches)) {
            if(Math.abs(encoderValToInches(robot.fr.getCurrentPosition()-startRightEncoderPos)) > inches){
                robot.fr.setPower(0);
                robot.br.setPower(0);
            }
            if(Math.abs(encoderValToInches(robot.fl.getCurrentPosition()-startLeftEncoderPos)) > inches){
                robot.fl.setPower(0);
                robot.bl.setPower(0);
            }
        }
        robot.allStop();
    }
//
//    public void turnEncoders(double power, double val){
//        int startLeftEncoderPos = robot.fl.getCurrentPosition();
//        robot.fl.setPower(-power);
//        robot.fr.setPower(power);
//        robot.bl.setPower(-power);
//        robot.br.setPower(power);
//        while(opModeIsActive() && opModeIsActive() && Math.abs(robot.fl.getCurrentPosition()-startLeftEncoderPos) < val) {
//        }
//        robot.allStop();
//    }
//
//    public void moveBackwards(double power, double inches){
//        int startLeftEncoderPos = robot.fl.getCurrentPosition();
//        int startRightEncoderPos = robot.fr.getCurrentPosition();
//        robot.fl.setPower(power);
//        robot.fr.setPower(power);
//        robot.bl.setPower(power);
//        robot.br.setPower(power);
//        while(opModeIsActive() && opModeIsActive() && encoderValToInches(robot.fl.getCurrentPosition()-startLeftEncoderPos) > -inches || encoderValToInches(robot.fr.getCurrentPosition()-startRightEncoderPos) > -inches) {
//            if (encoderValToInches(robot.fl.getCurrentPosition() - startLeftEncoderPos) <= -inches) {
//                robot.fl.setPower(0);
//                robot.bl.setPower(0);
//            }
//            if (encoderValToInches(robot.fr.getCurrentPosition() - startLeftEncoderPos) <= -inches) {
//                robot.fr.setPower(0);
//                robot.br.setPower(0);
//            }
//        }
//        robot.allStop();
//    }
//
//    public void unlockHang(){
//        robot.lift.setPower(-0.5);
//        robot.hangRelease.setPosition(0);
//        sleep(500);
//        robot.lift.setPower(0);
//        sleep(1000); // wait for locker to move completely out of way before drop
//    }
//
//    public void lowerHangPowerless(int timeMillis){
//        robot.lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        sleep(timeMillis);
//        robot.lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//    }
//
//    public void detachHang(){
//        while(opModeIsActive() && opModeIsActive() && robot.lift.getCurrentPosition() > -4684){
//            robot.lift.setPower(0.5);
//        }
//        robot.lift.setPower(0);
//        robot.hookRelease.setPosition(0.2); //0.13 correct
//        sleep(750);
//        robot.hookRelease.setPosition(0.6);
//        sleep(750);
//        robot.hookRelease.setPosition(0.2);
//        sleep(750);
//        robot.hookRelease.setPosition(0.6);
//        sleep(750);
//        robot.hookRelease.setPosition(0.2);
//        sleep(1000);
//    }
//
//    public void resetHang(){
//        robot.hookRelease.setPosition(0.6);
//        while(opModeIsActive() && opModeIsActive() && robot.lift.getCurrentPosition() < -100){
//            robot.lift.setPower(-0.3);
//        }
//        robot.lift.setPower(0);
//    }
//
//    public void turn(double power, double degrees){ // positive power and positive degrees for right turn
//        int startHeading = robot.getHeading();
//        int headingDiff = robot.getHeading()-startHeading;
//        while(opModeIsActive() && Math.abs(headingDiff)<Math.abs(degrees)){
//            headingDiff = robot.getHeading()-startHeading;
//            robot.driveLimitless(power, -power);
//        }
//        robot.allStop();
//    }
//
//    public void turn(double power, double degrees, int startHeading){ // positive power and positive degrees for right turn
//        int headingDiff = robot.getHeading()-startHeading;
//        while(opModeIsActive() && Math.abs(headingDiff)<Math.abs(degrees)){
//            headingDiff = robot.getHeading()-startHeading;
//            robot.driveLimitless(power, -power);
//        }
//        robot.allStop();
//    }
//
//    public void dropMarker(){
//        robot.markerDeployer.setPosition(0);
//        sleep(1000);
//        robot.markerDeployer.setPosition(0.1);
//        sleep(100);
//        robot.markerDeployer.setPosition(0);
//        sleep(100);
//        robot.markerDeployer.setPosition(0.1);
//        sleep(100);
//        robot.markerDeployer.setPosition(0);
//        sleep(100);
//        robot.markerDeployer.setPosition(0.85);
//        sleep(1000);
//    }
}