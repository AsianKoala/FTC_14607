package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.Teleop.opencvSkystoneDetector;


import java.util.ArrayList;

@Deprecated
@Autonomous(name = "Blue Auto", group = "Firefly")
public class fckingstupid extends LinearOpMode {

    public static final String TAG = "Vuforia Navigation Sample";

    OpenGLMatrix lastLocation = null;

    VuforiaLocalizer vuforia;

    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftRear;
    private DcMotor rightRear;
    private DcMotor leftIntake;
    private DcMotor rightIntake;
    private DcMotor leftSlide;
    private DcMotor rightSlide;
    private Servo flipper, gripper, rotater, leftSlam, rightSlam;

    private ArrayList<DcMotor> driveMotors = new ArrayList<>();




    private final double flipperHome =  0.95;
    private final double flipperOut = 0.25;
    private final double flipperBetween = (flipperHome + flipperOut)/2;
    private final double rotaterHome = 0.279;
    private final double rotaterOut = 0.95;
    private final double gripperHome = 0.41;
    private final double gripperGrip = 0.2;

    private double oldSlideLeft = 0;
    private double oldSlideRight = 0;
    private double newSlideLeft = 0;
    private double newSlideRight = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        leftFront = hardwareMap.get(DcMotor.class, "FL");
        leftRear = hardwareMap.get(DcMotor.class, "BL");
        rightRear = hardwareMap.get(DcMotor.class, "FR");
        rightFront = hardwareMap.get(DcMotor.class, "BR");
        leftIntake = hardwareMap.get(DcMotor.class, "leftIntake");
        rightIntake = hardwareMap.get(DcMotor.class, "rightIntake");
        leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");

        gripper = hardwareMap.get(Servo.class, "gripper");
        flipper = hardwareMap.get(Servo.class, "flipper");
        rotater = hardwareMap.get(Servo.class, "rotater");
        leftSlam = hardwareMap.get(Servo.class, "leftSlam");
        rightSlam = hardwareMap.get(Servo.class, "rightSlam");


        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        rightIntake.setDirection(DcMotor.Direction.REVERSE);
        leftSlide.setDirection(DcMotor.Direction.REVERSE);

        leftSlide.setTargetPosition(0);
        rightSlide.setTargetPosition(0);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        driveMotors.add(leftRear);
        driveMotors.add(leftFront);
        driveMotors.add(rightFront);
        driveMotors.add(rightRear);

        double yPower = 0;
        double xPower = 0;
        double zPower = 0;

        telemetry.addData("Ready.", 0);
        telemetry.update();

        waitForStart();

        sleep(500);

        xPower = -0.5;
        yPower = 0;
        zPower = 0;
        driveMecanum(xPower, yPower, zPower);
        sleep(1000);

        xPower = 0;
        yPower = 0.5;
        zPower = 0;
        driveMecanum(xPower, yPower, zPower);
        sleep(1000);

        xPower = 0;
        yPower = 0;
        zPower = 0;
        driveMecanum(xPower, yPower, zPower);
        sleep(1000);


        grabFoundation();
        sleep(2000);

        xPower = 0;
        yPower = -0.5;
        zPower = 0;
        driveMecanum(xPower, yPower, zPower);
        sleep(3000);

        ungrabFoundation();

        xPower = 0;
        yPower = 0;
        zPower = 0;
        driveMecanum(xPower, yPower, zPower);
        sleep(1000);

        xPower = 0.5;
        yPower = 0;
        zPower = 0;
        driveMecanum(xPower, yPower, zPower);
        sleep(2600);

        xPower = 0;
        yPower = 0;
        zPower = 0;
        driveMecanum(xPower, yPower, zPower);
        sleep(1000);

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

}