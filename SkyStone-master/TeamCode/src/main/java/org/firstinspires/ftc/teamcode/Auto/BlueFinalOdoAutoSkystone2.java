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
import org.firstinspires.ftc.teamcode.HelperClasses.GLOBALS;
import org.firstinspires.ftc.teamcode.Teleop.FireFlyRobot;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import static org.firstinspires.ftc.teamcode.HelperClasses.GLOBALS.backClawGripperOpen;
import static org.firstinspires.ftc.teamcode.HelperClasses.GLOBALS.ourSkystonePosition;


@Autonomous(name = "REAL Blue Odo Auto Skystone", group = "Firefly")
public class BlueFinalOdoAutoSkystone2 extends LinearOpMode {


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


    private static float[] midPos = {5.8f/8f, 5f/8f};//0 = col, 1 = row
    private static float[] leftPos = {4.1f/8f, 5f/8f}; //2.7f/8f
    private static float[] rightPos = {7.5f/8f, 5f/8f};
//
//    private static float[] midPos = {4.3f/8f, 2.0f/8f};//0 = col, 1 = row
//    private static float[] leftPos = {2.3f/8f, 2.0f/8f}; //2.7f/8f
//    private static float[] rightPos = {6.3f/8f, 2.0f/8f};
    //moves all rectangles right or left by amount. units are in ratio to monitor

    private final int rows = 640;
    private final int cols = 480;

    OpenCvCamera phoneCam;

    boolean angleCorrectFront = true;

    private BNO055IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        robot.init(hardwareMap);
        robot.initPositions();

        // initialize claw

        robot.clawGripper.setPosition(GLOBALS.backClawGripperClosed);
        robot.clawFlipper.setPosition(GLOBALS.backClawFlipperHome);
        robot.clawRotater.setPosition(GLOBALS.backClawRotaterHome);

        // end init claw

        robot.setDriveMotorsVelocityMode();

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);


        BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN);


//
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();//open camera
        phoneCam.setPipeline(new StageSwitchingPipeline());//different stages
        phoneCam.startStreaming(rows, cols, OpenCvCameraRotation.SIDEWAYS_LEFT);//display on RC
        //width, height
        //width = height in this case, because camera is in portrait mode.
//
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
            telemetry.addData("Ready.", 0);
            telemetry.addData("heading: ", getHeadingRaw180(startHeading));
            telemetry.update();
        }

        startHeading = getHeadingRaw180(0);
        lastAngleFound = startHeading;
        phoneCam.closeCameraDevice();

        int state = 1;

        double flpower = 0;
        double frpower = 0;
        double blpower = 0;
        double brpower = 0;

        xOdoStartVal = xOdoVal();
        yOdoStartVal = yOdoVal();

        long stateStartTime = System.currentTimeMillis();

//        //////////////////////////
//
//
//        // FOR DEBUG ONLY !!!
//
//        ourSkystonePosition = GLOBALS.SKYSTONE_POSITION.RIGHT;
//
//        /////////////////////////

        HashMap<String, Double> powers = new HashMap<String, Double>();
        powers.put("fl", 0.0);
        powers.put("fr", 0.0);
        powers.put("bl", 0.0);
        powers.put("br", 0.0);

        while(opModeIsActive()) {

            powers.put("fl", 0.0);
            powers.put("fr", 0.0);
            powers.put("bl", 0.0);
            powers.put("br", 0.0);

            if(ourSkystonePosition == GLOBALS.SKYSTONE_POSITION.LEFT) {
                if(state == 1) {
                    powers = moveToPositionAbsolute(25, 8, 0.25, stateStartTime);
                    if(powers.get("done") != 1.0) {

                        if(xOdoVal() > 2) {
                            robot.clawFlipper.setPosition(GLOBALS.backClawFlipperAntiHome);
                        }

                        if(xOdoVal() > 10) {
                            robot.clawRotater.setPosition(GLOBALS.backClawRotaterOut);
                        }

                        if(xOdoVal() > 20) {
                            robot.clawGripper.setPosition(GLOBALS.backClawGripperOpen);
                        }

                    } else {
                        state = 5;
                        stateStartTime = System.currentTimeMillis();

                    }
                }

                if(state == 5) {
                    if(System.currentTimeMillis() - stateStartTime < 300) {
                        robot.clawFlipper.setPosition(GLOBALS.backClawFlipperDown);
                    } else {
                        state = 6;
                        stateStartTime = System.currentTimeMillis();
                    }
                }

                if(state == 6) {
                    if(System.currentTimeMillis() - stateStartTime < 200) {
                        robot.clawGripper.setPosition(GLOBALS.backClawGripperClosed);
                    } else {
                        state = 7;
                        stateStartTime = System.currentTimeMillis();
                    }
                }

                if(state == 7) {
                    if(System.currentTimeMillis() - stateStartTime < 300) { //400
                        robot.clawFlipper.setPosition(GLOBALS.backClawFlipperUp);
                    } else {
                        state = 8;
                        stateStartTime = System.currentTimeMillis();
                    }
                }

                if(state == 8) {
                    powers = moveToPositionAbsolute(24, 75, 1.5, stateStartTime); // 24 x 0.75
                    if(powers.get("done") != 1.0) {
                        if(yOdoVal() > 5) {
                            robot.clawRotater.setPosition(GLOBALS.backClawRotaterBack);
                        }
                        if(yOdoVal() > 20) {
                            robot.clawFlipper.setPosition(GLOBALS.backClawFlipperTravel);
                        }

                        if(yOdoVal() > 70) {
                            robot.clawRotater.setPosition(GLOBALS.backClawRotaterOut);
                            robot.clawFlipper.setPosition(GLOBALS.backClawFlipperUp);
                        }
                    } else {
                        state = 9;
                        stateStartTime = System.currentTimeMillis();

                    }
                }

                if(state == 9) {
                    if(System.currentTimeMillis() - stateStartTime < 150) { //250
                        robot.clawRotater.setPosition(GLOBALS.backClawRotaterBack);
                        robot.clawFlipper.setPosition(GLOBALS.backClawFlipperTravel);
                        robot.clawRotater.setPosition(GLOBALS.backClawRotaterOut);
                        robot.clawFlipper.setPosition(GLOBALS.backClawFlipperUp);

                        robot.clawGripper.setPosition(GLOBALS.backClawGripperOpen);
                    } else {
                        state = 10;
                        stateStartTime = System.currentTimeMillis();
                    }
                }

                if(state == 10) {
                    powers = moveToPositionAbsolute(24-1, -16, 0.25, stateStartTime); //0.25
                    if(powers.get("done") != 1.0) {
                        if(yOdoVal() < 72) {
                            robot.clawRotater.setPosition(GLOBALS.backClawRotaterBack);
                        }
                        if(yOdoVal() < 70) {
                            robot.clawFlipper.setPosition(GLOBALS.backClawFlipperTravel);
                        }

                        if(yOdoVal() < 0) {
                            robot.clawRotater.setPosition(GLOBALS.backClawRotaterOut);
                            robot.clawFlipper.setPosition(GLOBALS.backClawFlipperUp);
                        }
                    } else {
                        state = 11;
                        stateStartTime = System.currentTimeMillis();

                    }
                }

                if(state == 11) {
                    if(System.currentTimeMillis() - stateStartTime < 150) { // 300
                        robot.clawRotater.setPosition(GLOBALS.backClawRotaterOut);
                        robot.clawFlipper.setPosition(GLOBALS.backClawFlipperDown);

                        robot.clawGripper.setPosition(GLOBALS.backClawGripperOpen);
                    } else {
                        state = 12;
                        stateStartTime = System.currentTimeMillis();
                    }
                }

                if(state == 12) {
                    if(System.currentTimeMillis() - stateStartTime < 200) {
                        robot.clawGripper.setPosition(GLOBALS.backClawGripperClosed);
                    } else {
                        state = 13;
                        stateStartTime = System.currentTimeMillis();
                    }
                }

                if(state == 13) {
                    if(System.currentTimeMillis() - stateStartTime < 300) { //400
                        robot.clawFlipper.setPosition(GLOBALS.backClawFlipperUp);
                    } else {
                        state = 14;
                        stateStartTime = System.currentTimeMillis();
                    }
                }

                if(state == 14) {
                    powers = moveToPositionAbsolute(24, 80, 1.25, stateStartTime); // y 75 85 too far 0.75
                    if(powers.get("done") != 1.0) {
                        if(yOdoVal() > 5) {
                            robot.clawRotater.setPosition(GLOBALS.backClawRotaterBack);
                        }
                        if(yOdoVal() > 20) {
                            robot.clawFlipper.setPosition(GLOBALS.backClawFlipperTravel);
                        }

                        if(yOdoVal() > 70) {
                            robot.clawRotater.setPosition(GLOBALS.backClawRotaterOut);
                            robot.clawFlipper.setPosition(GLOBALS.backClawFlipperUp);
                        }
                    } else {
                        state = 15;
                        stateStartTime = System.currentTimeMillis();

                    }
                }

                if(state == 15) {
                    if(System.currentTimeMillis() - stateStartTime < 150) { //250
                        robot.clawRotater.setPosition(GLOBALS.backClawRotaterBack);
                        robot.clawFlipper.setPosition(GLOBALS.backClawFlipperTravel);
                        robot.clawRotater.setPosition(GLOBALS.backClawRotaterOut);
                        robot.clawFlipper.setPosition(GLOBALS.backClawFlipperUp);

                        robot.clawGripper.setPosition(GLOBALS.backClawGripperOpen);
                    } else {
                        state = 16;
                        stateStartTime = System.currentTimeMillis();
                    }
                }

                if(state == 16) {
                    angleCorrectFront = false;
                    powers = rotateAbsolute(-90, 0.5, stateStartTime, 3000);
                    if(powers.get("done") != 1.0) {

                        if(getHeadingRaw180(startHeading) < -5) {
                            robot.clawGripper.setPosition(GLOBALS.backClawGripperClosed);
                            robot.clawFlipper.setPosition(GLOBALS.backClawFlipperAntiHome);
                        }

                        if(getHeadingRaw180(startHeading) < -40) {
                            robot.clawRotater.setPosition(GLOBALS.backClawRotaterHome);
                        }

                        if(getHeadingRaw180(startHeading) < -80) {
                            robot.clawFlipper.setPosition(GLOBALS.backClawFlipperHome);
                        }

                    } else {
                        state = 17;
                        stateStartTime = System.currentTimeMillis();

                    }
                }

                if(state == 17) {
                    if(System.currentTimeMillis() - stateStartTime < 25) {
                        robot.clawFlipper.setPosition(GLOBALS.backClawFlipperHome);
                        robot.clawRotater.setPosition(GLOBALS.backClawRotaterHome);
                        robot.clawGripper.setPosition(GLOBALS.backClawGripperClosed);
                    } else {
                        state = 18;
                        stateStartTime = System.currentTimeMillis();


                        xOdoStartVal = xOdoVal();
                        yOdoStartVal = yOdoVal();

                        robot.setDriveMotorsNoVelMode();
                    }
                }

                if(state == 18) {
                    if(System.currentTimeMillis() - stateStartTime < 600) {
                        powers.put("fl", -0.3);
                        powers.put("fr", -0.3);
                        powers.put("bl", -0.3);
                        powers.put("br", -0.3);
                    } else {
                        state = 19;
                        stateStartTime = System.currentTimeMillis();
                    }
                }

                if(state == 19) {
                    if(System.currentTimeMillis() - stateStartTime < 600) {
                        robot.setFoundationHookGrip();
                    } else {
                        state = 20;
                        stateStartTime = System.currentTimeMillis();
                    }
                }

                if(state == 20) {
                    if(System.currentTimeMillis() - stateStartTime < 1000) { //1500
                        powers.put("fl", 0.5);
                        powers.put("fr", 0.5);
                        powers.put("bl", 0.5);
                        powers.put("br", 0.5);
                    } else {
                        state = 21;
                        stateStartTime = System.currentTimeMillis();
                    }
                }

                if(state == 21) {
                    angleCorrectFront = false;
                    powers = rotateAbsolute(-182, 5, stateStartTime, 3000);
                    powers.put("fl", -0.1);
                    powers.put("bl", -0.1);
                    if(powers.get("done") != 1.0) {
                        if(getHeadingRaw180(startHeading) < -180) {
                            robot.setFoundationHookHome();
                        }

                    } else {
                        state = 22;
                        stateStartTime = System.currentTimeMillis();
                        robot.setFoundationHookHome();

                    }
                }

                if(state == 22) {
                    if(System.currentTimeMillis() - stateStartTime < 100) {
                        robot.setFoundationHookHome();
                    } else {
                        state = 23;
                        stateStartTime = System.currentTimeMillis();
                    }
                }

                if(state == 23) {
                    if(System.currentTimeMillis() - stateStartTime < 1700) {
                        powers.put("fl", -0.1);
                        powers.put("fr", 0.99);
                        powers.put("bl", -0.1);
                        powers.put("br", 0.99);
                        if(System.currentTimeMillis() - stateStartTime > 700) {
                            powers.put("fl", 0.99);
                            powers.put("fr", 0.1);
                            powers.put("bl", 0.99);
                            powers.put("br", 0.1);
                        }
                    } else {
                        state = 24;
                        stateStartTime = System.currentTimeMillis();
                    }
                }
            }

            if(ourSkystonePosition == GLOBALS.SKYSTONE_POSITION.MIDDLE) {
                if(state == 1) {
                    powers = moveToPositionAbsolute(25, 0, 0.25, stateStartTime);
                    if(powers.get("done") != 1.0) {

                        if(xOdoVal() > 2) {
                            robot.clawFlipper.setPosition(GLOBALS.backClawFlipperAntiHome);
                        }

                        if(xOdoVal() > 10) {
                            robot.clawRotater.setPosition(GLOBALS.backClawRotaterOut);
                        }

                        if(xOdoVal() > 20) {
                            robot.clawGripper.setPosition(GLOBALS.backClawGripperOpen);
                        }

                    } else {
                        state = 5;
                        stateStartTime = System.currentTimeMillis();

                    }
                }

                if(state == 5) {
                    if(System.currentTimeMillis() - stateStartTime < 300) {
                        robot.clawFlipper.setPosition(GLOBALS.backClawFlipperDown);
                    } else {
                        state = 6;
                        stateStartTime = System.currentTimeMillis();
                    }
                }

                if(state == 6) {
                    if(System.currentTimeMillis() - stateStartTime < 200) {
                        robot.clawGripper.setPosition(GLOBALS.backClawGripperClosed);
                    } else {
                        state = 7;
                        stateStartTime = System.currentTimeMillis();
                    }
                }

                if(state == 7) {
                    if(System.currentTimeMillis() - stateStartTime < 300) { //400
                        robot.clawFlipper.setPosition(GLOBALS.backClawFlipperUp);
                    } else {
                        state = 8;
                        stateStartTime = System.currentTimeMillis();
                    }
                }

                if(state == 8) {
                    powers = moveToPositionAbsolute(24, 75, 1.5, stateStartTime); // 24 x
                    if(powers.get("done") != 1.0) {
                        if(yOdoVal() > 5) {
                            robot.clawRotater.setPosition(GLOBALS.backClawRotaterBack);
                        }
                        if(yOdoVal() > 20) {
                            robot.clawFlipper.setPosition(GLOBALS.backClawFlipperTravel);
                        }

                        if(yOdoVal() > 70) {
                            robot.clawRotater.setPosition(GLOBALS.backClawRotaterOut);
                            robot.clawFlipper.setPosition(GLOBALS.backClawFlipperUp);
                        }
                    } else {
                        state = 9;
                        stateStartTime = System.currentTimeMillis();

                    }
                }

                if(state == 9) {
                    if(System.currentTimeMillis() - stateStartTime < 150) { //250
                        robot.clawRotater.setPosition(GLOBALS.backClawRotaterBack);
                        robot.clawFlipper.setPosition(GLOBALS.backClawFlipperTravel);
                        robot.clawRotater.setPosition(GLOBALS.backClawRotaterOut);
                        robot.clawFlipper.setPosition(GLOBALS.backClawFlipperUp);

                        robot.clawGripper.setPosition(GLOBALS.backClawGripperOpen);
                    } else {
                        state = 10;
                        stateStartTime = System.currentTimeMillis();
                    }
                }

                if(state == 10) {
                    powers = moveToPositionAbsolute(24-1, -24, 0.25, stateStartTime);
                    if(powers.get("done") != 1.0) {
                        if(yOdoVal() < 72) {
                            robot.clawRotater.setPosition(GLOBALS.backClawRotaterBack);
                        }
                        if(yOdoVal() < 70) {
                            robot.clawFlipper.setPosition(GLOBALS.backClawFlipperTravel);
                        }

                        if(yOdoVal() < 0) {
                            robot.clawRotater.setPosition(GLOBALS.backClawRotaterOut);
                            robot.clawFlipper.setPosition(GLOBALS.backClawFlipperUp);
                        }
                    } else {
                        state = 11;
                        stateStartTime = System.currentTimeMillis();

                    }
                }

                if(state == 11) {
                    if(System.currentTimeMillis() - stateStartTime < 150) { // 300
                        robot.clawRotater.setPosition(GLOBALS.backClawRotaterOut);
                        robot.clawFlipper.setPosition(GLOBALS.backClawFlipperDown);

                        robot.clawGripper.setPosition(GLOBALS.backClawGripperOpen);
                    } else {
                        state = 12;
                        stateStartTime = System.currentTimeMillis();
                    }
                }

                if(state == 12) {
                    if(System.currentTimeMillis() - stateStartTime < 200) {
                        robot.clawGripper.setPosition(GLOBALS.backClawGripperClosed);
                    } else {
                        state = 13;
                        stateStartTime = System.currentTimeMillis();
                    }
                }

                if(state == 13) {
                    if(System.currentTimeMillis() - stateStartTime < 300) { //400
                        robot.clawFlipper.setPosition(GLOBALS.backClawFlipperUp);
                    } else {
                        state = 14;
                        stateStartTime = System.currentTimeMillis();
                    }
                }

                if(state == 14) {
                    powers = moveToPositionAbsolute(24, 80, 1.5, stateStartTime); // y 75 85 too far
                    if(powers.get("done") != 1.0) {
                        if(yOdoVal() > 5) {
                            robot.clawRotater.setPosition(GLOBALS.backClawRotaterBack);
                        }
                        if(yOdoVal() > 20) {
                            robot.clawFlipper.setPosition(GLOBALS.backClawFlipperTravel);
                        }

                        if(yOdoVal() > 70) {
                            robot.clawRotater.setPosition(GLOBALS.backClawRotaterOut);
                            robot.clawFlipper.setPosition(GLOBALS.backClawFlipperUp);
                        }
                    } else {
                        state = 15;
                        stateStartTime = System.currentTimeMillis();

                    }
                }

                if(state == 15) {
                    if(System.currentTimeMillis() - stateStartTime < 150) { //250
                        robot.clawRotater.setPosition(GLOBALS.backClawRotaterBack);
                        robot.clawFlipper.setPosition(GLOBALS.backClawFlipperTravel);
                        robot.clawRotater.setPosition(GLOBALS.backClawRotaterOut);
                        robot.clawFlipper.setPosition(GLOBALS.backClawFlipperUp);

                        robot.clawGripper.setPosition(GLOBALS.backClawGripperOpen);
                    } else {
                        state = 16;
                        stateStartTime = System.currentTimeMillis();
                    }
                }

                if(state == 16) {
                    angleCorrectFront = false;
                    powers = rotateAbsolute(-90, 0.5, stateStartTime, 3000);
                    if(powers.get("done") != 1.0) {

                        if(getHeadingRaw180(startHeading) < -5) {
                            robot.clawGripper.setPosition(GLOBALS.backClawGripperClosed);
                            robot.clawFlipper.setPosition(GLOBALS.backClawFlipperAntiHome);
                        }

                        if(getHeadingRaw180(startHeading) < -40) {
                            robot.clawRotater.setPosition(GLOBALS.backClawRotaterHome);
                        }

                        if(getHeadingRaw180(startHeading) < -80) {
                            robot.clawFlipper.setPosition(GLOBALS.backClawFlipperHome);
                        }

                    } else {
                        state = 17;
                        stateStartTime = System.currentTimeMillis();

                    }
                }

                if(state == 17) {
                    if(System.currentTimeMillis() - stateStartTime < 25) {
                        robot.clawFlipper.setPosition(GLOBALS.backClawFlipperHome);
                        robot.clawRotater.setPosition(GLOBALS.backClawRotaterHome);
                        robot.clawGripper.setPosition(GLOBALS.backClawGripperClosed);
                    } else {
                        state = 18;
                        stateStartTime = System.currentTimeMillis();


                        xOdoStartVal = xOdoVal();
                        yOdoStartVal = yOdoVal();

                        robot.setDriveMotorsNoVelMode();
                    }
                }

                if(state == 18) {
                    if(System.currentTimeMillis() - stateStartTime < 600) {
                        powers.put("fl", -0.3);
                        powers.put("fr", -0.3);
                        powers.put("bl", -0.3);
                        powers.put("br", -0.3);
                    } else {
                        state = 19;
                        stateStartTime = System.currentTimeMillis();
                    }
                }

                if(state == 19) {
                    if(System.currentTimeMillis() - stateStartTime < 600) {
                        robot.setFoundationHookGrip();
                    } else {
                        state = 20;
                        stateStartTime = System.currentTimeMillis();
                    }
                }

                if(state == 20) {
                    if(System.currentTimeMillis() - stateStartTime < 1000) { //1500
                        powers.put("fl", 0.5);
                        powers.put("fr", 0.5);
                        powers.put("bl", 0.5);
                        powers.put("br", 0.5);
                    } else {
                        state = 21;
                        stateStartTime = System.currentTimeMillis();
                    }
                }

                if(state == 21) {
                    angleCorrectFront = false;
                    powers = rotateAbsolute(-182, 5, stateStartTime, 3000);
                    powers.put("fl", -0.1);
                    powers.put("bl", -0.1);
                    if(powers.get("done") != 1.0) {
                        if(getHeadingRaw180(startHeading) < -180) {
                            robot.setFoundationHookHome();
                        }

                    } else {
                        state = 22;
                        stateStartTime = System.currentTimeMillis();
                        robot.setFoundationHookHome();

                    }
                }

                if(state == 22) {
                    if(System.currentTimeMillis() - stateStartTime < 100) {
                        robot.setFoundationHookHome();
                    } else {
                        state = 23;
                        stateStartTime = System.currentTimeMillis();
                    }
                }

                if(state == 23) {
                    if(System.currentTimeMillis() - stateStartTime < 1700) {
                        powers.put("fl", -0.1);
                        powers.put("fr", 0.99);
                        powers.put("bl", -0.1);
                        powers.put("br", 0.99);
                        if(System.currentTimeMillis() - stateStartTime > 700) {
                            powers.put("fl", 0.99);
                            powers.put("fr", 0.1);
                            powers.put("bl", 0.99);
                            powers.put("br", 0.1);
                        }
                    } else {
                        state = 24;
                        stateStartTime = System.currentTimeMillis();
                    }
                }
            }

            if(ourSkystonePosition == GLOBALS.SKYSTONE_POSITION.RIGHT) {
                if(state == 1) {
                    powers = moveToPositionAbsolute(25, -8, 0.25, stateStartTime);
                    if(powers.get("done") != 1.0) {

                        if(xOdoVal() > 2) {
                            robot.clawFlipper.setPosition(GLOBALS.backClawFlipperAntiHome);
                        }

                        if(xOdoVal() > 10) {
                            robot.clawRotater.setPosition(GLOBALS.backClawRotaterOut);
                        }

                        if(xOdoVal() > 20) {
                            robot.clawGripper.setPosition(GLOBALS.backClawGripperOpen);
                        }

                    } else {
                        state = 5;
                        stateStartTime = System.currentTimeMillis();

                    }
                }

                if(state == 5) {
                    if(System.currentTimeMillis() - stateStartTime < 300) {
                        robot.clawFlipper.setPosition(GLOBALS.backClawFlipperDown);
                    } else {
                        state = 6;
                        stateStartTime = System.currentTimeMillis();
                    }
                }

                if(state == 6) {
                    if(System.currentTimeMillis() - stateStartTime < 200) {
                        robot.clawGripper.setPosition(GLOBALS.backClawGripperClosed);
                    } else {
                        state = 7;
                        stateStartTime = System.currentTimeMillis();
                    }
                }

                if(state == 7) {
                    if(System.currentTimeMillis() - stateStartTime < 300) { //400
                        robot.clawFlipper.setPosition(GLOBALS.backClawFlipperUp);
                    } else {
                        state = 8;
                        stateStartTime = System.currentTimeMillis();
                    }
                }

                if(state == 8) {
                    powers = moveToPositionAbsolute(24, 75, 1.5, stateStartTime); // 24 x
                    if(powers.get("done") != 1.0) {
                        if(yOdoVal() > 5) {
                            robot.clawRotater.setPosition(GLOBALS.backClawRotaterBack);
                        }
                        if(yOdoVal() > 20) {
                            robot.clawFlipper.setPosition(GLOBALS.backClawFlipperTravel);
                        }

                        if(yOdoVal() > 70) {
                            robot.clawRotater.setPosition(GLOBALS.backClawRotaterOut);
                            robot.clawFlipper.setPosition(GLOBALS.backClawFlipperUp);
                        }
                    } else {
                        state = 9;
                        stateStartTime = System.currentTimeMillis();

                    }
                }

                if(state == 9) {
                    if(System.currentTimeMillis() - stateStartTime < 150) { //250
                        robot.clawRotater.setPosition(GLOBALS.backClawRotaterBack);
                        robot.clawFlipper.setPosition(GLOBALS.backClawFlipperTravel);
                        robot.clawRotater.setPosition(GLOBALS.backClawRotaterOut);
                        robot.clawFlipper.setPosition(GLOBALS.backClawFlipperUp);

                        robot.clawGripper.setPosition(GLOBALS.backClawGripperOpen);
                    } else {
                        state = 10;
                        stateStartTime = System.currentTimeMillis();
                    }
                }

                if(state == 10) {
                    powers = moveToPositionAbsolute(24-1, -32, 0.25, stateStartTime);
                    if(powers.get("done") != 1.0) {
                        if(yOdoVal() < 72) {
                            robot.clawRotater.setPosition(GLOBALS.backClawRotaterBack);
                        }
                        if(yOdoVal() < 70) {
                            robot.clawFlipper.setPosition(GLOBALS.backClawFlipperTravel);
                        }

                        if(yOdoVal() < 0) {
                            robot.clawRotater.setPosition(GLOBALS.backClawRotaterOut);
                            robot.clawFlipper.setPosition(GLOBALS.backClawFlipperUp);
                        }
                    } else {
                        state = 11;
                        stateStartTime = System.currentTimeMillis();

                    }
                }

                if(state == 11) {
                    if(System.currentTimeMillis() - stateStartTime < 150) { // 300
                        robot.clawRotater.setPosition(GLOBALS.backClawRotaterOut);
                        robot.clawFlipper.setPosition(GLOBALS.backClawFlipperDown);

                        robot.clawGripper.setPosition(GLOBALS.backClawGripperOpen);
                    } else {
                        state = 12;
                        stateStartTime = System.currentTimeMillis();
                    }
                }

                if(state == 12) {
                    if(System.currentTimeMillis() - stateStartTime < 200) {
                        robot.clawGripper.setPosition(GLOBALS.backClawGripperClosed);
                    } else {
                        state = 13;
                        stateStartTime = System.currentTimeMillis();
                    }
                }

                if(state == 13) {
                    if(System.currentTimeMillis() - stateStartTime < 300) { //400
                        robot.clawFlipper.setPosition(GLOBALS.backClawFlipperUp);
                    } else {
                        state = 14;
                        stateStartTime = System.currentTimeMillis();
                    }
                }

                if(state == 14) {
                    powers = moveToPositionAbsolute(24, 80, 1.5, stateStartTime); // y 75 85 too far
                    if(powers.get("done") != 1.0) {
                        if(yOdoVal() > 5) {
                            robot.clawRotater.setPosition(GLOBALS.backClawRotaterBack);
                        }
                        if(yOdoVal() > 20) {
                            robot.clawFlipper.setPosition(GLOBALS.backClawFlipperTravel);
                        }

                        if(yOdoVal() > 70) {
                            robot.clawRotater.setPosition(GLOBALS.backClawRotaterOut);
                            robot.clawFlipper.setPosition(GLOBALS.backClawFlipperUp);
                        }
                    } else {
                        state = 15;
                        stateStartTime = System.currentTimeMillis();

                    }
                }

                if(state == 15) {
                    if(System.currentTimeMillis() - stateStartTime < 150) { //250
                        robot.clawRotater.setPosition(GLOBALS.backClawRotaterBack);
                        robot.clawFlipper.setPosition(GLOBALS.backClawFlipperTravel);
                        robot.clawRotater.setPosition(GLOBALS.backClawRotaterOut);
                        robot.clawFlipper.setPosition(GLOBALS.backClawFlipperUp);

                        robot.clawGripper.setPosition(GLOBALS.backClawGripperOpen);
                    } else {
                        state = 16;
                        stateStartTime = System.currentTimeMillis();
                    }
                }

                if(state == 16) {
                    angleCorrectFront = false;
                    powers = rotateAbsolute(-90, 0.5, stateStartTime, 3000);
                    if(powers.get("done") != 1.0) {

                        if(getHeadingRaw180(startHeading) < -5) {
                            robot.clawGripper.setPosition(GLOBALS.backClawGripperClosed);
                            robot.clawFlipper.setPosition(GLOBALS.backClawFlipperAntiHome);
                        }

                        if(getHeadingRaw180(startHeading) < -40) {
                            robot.clawRotater.setPosition(GLOBALS.backClawRotaterHome);
                        }

                        if(getHeadingRaw180(startHeading) < -80) {
                            robot.clawFlipper.setPosition(GLOBALS.backClawFlipperHome);
                        }

                    } else {
                        state = 17;
                        stateStartTime = System.currentTimeMillis();

                    }
                }

                if(state == 17) {
                    if(System.currentTimeMillis() - stateStartTime < 25) {
                        robot.clawFlipper.setPosition(GLOBALS.backClawFlipperHome);
                        robot.clawRotater.setPosition(GLOBALS.backClawRotaterHome);
                        robot.clawGripper.setPosition(GLOBALS.backClawGripperClosed);
                    } else {
                        state = 18;
                        stateStartTime = System.currentTimeMillis();


                        xOdoStartVal = xOdoVal();
                        yOdoStartVal = yOdoVal();

                        robot.setDriveMotorsNoVelMode();
                    }
                }

                if(state == 18) {
                    if(System.currentTimeMillis() - stateStartTime < 600) {
                        powers.put("fl", -0.3);
                        powers.put("fr", -0.3);
                        powers.put("bl", -0.3);
                        powers.put("br", -0.3);
                    } else {
                        state = 19;
                        stateStartTime = System.currentTimeMillis();
                    }
                }

                if(state == 19) {
                    if(System.currentTimeMillis() - stateStartTime < 600) {
                        robot.setFoundationHookGrip();
                    } else {
                        state = 20;
                        stateStartTime = System.currentTimeMillis();
                    }
                }

                if(state == 20) {
                    if(System.currentTimeMillis() - stateStartTime < 1000) { //1500
                        powers.put("fl", 0.5);
                        powers.put("fr", 0.5);
                        powers.put("bl", 0.5);
                        powers.put("br", 0.5);
                    } else {
                        state = 21;
                        stateStartTime = System.currentTimeMillis();
                    }
                }

                if(state == 21) {
                    angleCorrectFront = false;
                    powers = rotateAbsolute(-182, 5, stateStartTime, 3000);
                    powers.put("fl", -0.1);
                    powers.put("bl", -0.1);
                    if(powers.get("done") != 1.0) {
                        if(getHeadingRaw180(startHeading) < -180) {
                            robot.setFoundationHookHome();
                        }

                    } else {
                        state = 22;
                        stateStartTime = System.currentTimeMillis();
                        robot.setFoundationHookHome();

                    }
                }

                if(state == 22) {
                    if(System.currentTimeMillis() - stateStartTime < 100) {
                        robot.setFoundationHookHome();
                    } else {
                        state = 23;
                        stateStartTime = System.currentTimeMillis();
                    }
                }

                if(state == 23) {
                    if(System.currentTimeMillis() - stateStartTime < 1700) {
                        powers.put("fl", -0.1);
                        powers.put("fr", 0.99);
                        powers.put("bl", -0.1);
                        powers.put("br", 0.99);
                        if(System.currentTimeMillis() - stateStartTime > 700) {
                            powers.put("fl", 0.99);
                            powers.put("fr", 0.1);
                            powers.put("bl", 0.99);
                            powers.put("br", 0.1);
                        }
                    } else {
                        state = 24;
                        stateStartTime = System.currentTimeMillis();
                    }
                }
            }


            if(angleCorrectFront) {
                if(Math.abs(getHeadingRaw180(startHeading)) > 1){
                    powers.put("fr", powers.get("fr")+(getHeadingRaw180(startHeading)) * 0.03); // 0.03
                    powers.put("br", powers.get("br")+(getHeadingRaw180(startHeading)) * 0.03);
                    powers.put("fl", powers.get("fl")-(getHeadingRaw180(startHeading)) * 0.03);
                    powers.put("bl", powers.get("bl")-(getHeadingRaw180(startHeading)) * 0.03);
                }
            }

            if(state == 99) {
                flpower = 0;
                frpower = 0;
                blpower = 0;
                brpower = 0;
                telemetry.addData("done", 0);
            }



            robot.setFLPower(powers.get("fl"));
            robot.setFRPower(powers.get("fr"));
            robot.setBLPower(powers.get("bl"));
            robot.setBRPower(powers.get("br"));


            telemetry.addData("yOdoVal: ", yOdoVal());
            telemetry.addData("xOdoVal: ", xOdoVal());
            telemetry.addData("heading: ", getHeadingRaw180(startHeading));
            telemetry.addData("state: ", state);
            telemetry.addData("power fl: ", powers.get("fl"));
            telemetry.addData("power fr: ", powers.get("fr"));
            telemetry.addData("power bl: ", powers.get("bl"));
            telemetry.addData("power br: ", powers.get("br"));
            telemetry.update();
        }

        robot.setDriveStop();

    }

    public HashMap<String, Double> moveToPositionAbsolute(double x, double y, double threshold, double stateStartTime) {
        double timeSinceStart = System.currentTimeMillis() - stateStartTime;
        final double minPower = 0.1;
        final double maxPower = 0.9;
        final double kP = 0.05; // inches and motor power 0.05
        final double accelToMaxTime = 3000; // milliseconds to full accel

        double fl_y = 0;
        double fr_y = 0;
        double bl_y = 0;
        double br_y = 0;
        double fl_x = 0;
        double fr_x = 0;
        double bl_x = 0;
        double br_x = 0;

        HashMap<String, Double> powers = new HashMap<String, Double>();

        if(Math.abs(x-xOdoVal()) < threshold && Math.abs(y-yOdoVal()) < threshold) {
            powers.put("fl", 0.0);
            powers.put("fr", 0.0);
            powers.put("bl", 0.0);
            powers.put("br", 0.0);
            powers.put("done", 1.0);
            return powers;
        }

        double dx = x-xOdoVal();
        double dy = y-yOdoVal();

        double xScale = Math.abs(Math.cos(Math.atan2(dy, dx)));
        double yScale = Math.abs(Math.sin(Math.atan2(dy, dx)));

        fl_y = dy * kP * yScale;
        fr_y = dy * kP * yScale;
        bl_y = dy * kP * yScale;
        br_y = dy * kP * yScale;

        fl_x = dx * kP * xScale * Math.sqrt(2);
        fr_x = dx * kP * xScale * Math.sqrt(2);
        bl_x = -dx * kP * xScale * Math.sqrt(2); //
        br_x = -dx * kP * xScale * Math.sqrt(2);

        double fl = fl_y + fl_x;
        double fr = fr_y + fr_x;
        double bl = bl_y + bl_x;
        double br = br_y + br_x;

//        double maxTotalPower = Math.max(Math.abs(fl), Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br))));
//
//        fl /= maxTotalPower;
//        fr /= maxTotalPower;
//        bl /= maxTotalPower;
//        br /= maxTotalPower;

        double accelScale = Math.min(timeSinceStart, accelToMaxTime)/accelToMaxTime;
//        telemetry.addData("fl real: ", Math.abs(fl * accelScale));
//        telemetry.addData("fr real: ", Math.abs(fr * accelScale));
//        telemetry.addData("bl real: ", Math.abs(bl * accelScale));
//        telemetry.addData("br real: ", Math.abs(br * accelScale));
//        telemetry.addData("xscale real: ", xScale);
//        telemetry.addData("yscale real: ", yScale);
//        telemetry.addData("dx real: ", dx);
//        telemetry.addData("dy real: ", dy);
//        telemetry.addData("accelscale real: ", accelScale);

        fl = Math.signum(fl) * Math.min(Math.max(Math.abs(fl * accelScale), minPower), maxPower);
        fr = Math.signum(fr) * Math.min(Math.max(Math.abs(fr * accelScale), minPower), maxPower);
        bl = Math.signum(bl) * Math.min(Math.max(Math.abs(bl * accelScale), minPower), maxPower);
        br = Math.signum(br) * Math.min(Math.max(Math.abs(br * accelScale), minPower), maxPower);

        powers.put("fl", fl);
        powers.put("fr", fr);
        powers.put("bl", bl);
        powers.put("br", br);
        powers.put("done", 0.0);

        return powers;
    }

    public HashMap<String, Double> rotateAbsolute(double targetHeading, double threshold, double stateStartTime, double timeoutTime) {

        double timeSinceStart = System.currentTimeMillis() - stateStartTime;
        final double minPower = 0.2;
        final double maxPower = 0.8;
        final double kP = 0.03; // degrees and motor power
        final double accelToMaxTime = 1000; // milliseconds to full accel

        HashMap<String, Double> powers = new HashMap<String, Double>();

        if(Math.abs(getHeadingRaw180(startHeading)-targetHeading) < threshold || timeoutTime > System.currentTimeMillis()-timeSinceStart) {
            powers.put("fl", 0.0);
            powers.put("fr", 0.0);
            powers.put("bl", 0.0);
            powers.put("br", 0.0);
            powers.put("done", 1.0);
            return powers;
        }

        double dTheta = targetHeading-getHeadingRaw180(startHeading);

        double fl = dTheta*kP;
        double fr = -dTheta*kP;
        double bl = dTheta*kP;
        double br = -dTheta*kP;

        double accelScale = Math.min(timeSinceStart, accelToMaxTime)/accelToMaxTime;

        fl = Math.signum(fl) * Math.min(Math.max(Math.abs(fl * accelScale), minPower), maxPower);
        fr = Math.signum(fr) * Math.min(Math.max(Math.abs(fr * accelScale), minPower), maxPower);
        bl = Math.signum(bl) * Math.min(Math.max(Math.abs(bl * accelScale), minPower), maxPower);
        br = Math.signum(br) * Math.min(Math.max(Math.abs(br * accelScale), minPower), maxPower);

        powers.put("fl", fl);
        powers.put("fr", fr);
        powers.put("bl", bl);
        powers.put("br", br);
        powers.put("done", 0.0);

        return powers;
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