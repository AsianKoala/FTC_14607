package org.firstinspires.ftc.teamcode.roadrunner.teamcode.drive.mecanum;

import android.support.annotation.NonNull;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.teamcode.roadrunner.teamcode.util.AxesSigns;
import org.firstinspires.ftc.teamcode.roadrunner.teamcode.util.BNO055IMUUtil;
import org.firstinspires.ftc.teamcode.roadrunner.teamcode.util.LynxModuleUtil;
import org.firstinspires.ftc.teamcode.roadrunner.teamcode.util.LynxOptimizedI2cFactory;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.ExpansionHubServo;
import org.openftc.revextensions2.RevBulkData;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.teamcode.roadrunner.teamcode.drive.DriveConstants.encoderTicksToInches;

/*
 * Optimized mecanum drive implementation for REV ExHs. The time savings may significantly improve
 * trajectory following performance with moderate additional complexity.
 */

public class HouseFly extends SampleMecanumDriveBase {
    private ExpansionHubEx master, slave;
    private ExpansionHubMotor frontLeft, backLeft, backRight, frontRight;
    private ExpansionHubMotor intakeLeft, intakeRight;
    private ExpansionHubServo leftSlam, rightSlam;
    private ExpansionHubServo outtake;
    private ExpansionHubServo gripper;



    private ArrayList<ExpansionHubMotor> driveMotors = new ArrayList<>();
    private ArrayList<ExpansionHubMotor> leftMotors = new ArrayList<>();
    private ArrayList<ExpansionHubMotor> rightMotors = new ArrayList<>();
    private ArrayList<ExpansionHubMotor> intakeMotors = new ArrayList<>();

    private BNO055IMU imu;


    /**
     * variable declaration
     */

    private final double leftReload = 90;
    private final double rightReload = 90;
    private final double leftDown = 180;
    private final double rightDown = 0;
    private final double outtakeOutPosition = 180;
    private final double outtakeReadyPosition = 0;
    private final double gripperOnPosition = 0;
    private final double gripperOffPosition = 180;







    public HouseFly(HardwareMap hardwareMap) {
        super();

        driveMotors.add(frontLeft);
        driveMotors.add(frontRight);
        driveMotors.add(backLeft);
        driveMotors.add(backRight);
        leftMotors.add(frontLeft);
        leftMotors.add(backLeft);
        rightMotors.add(frontRight);
        rightMotors.add(backRight);
        intakeMotors.add(intakeLeft);
        intakeMotors.add(intakeRight);

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);


        // map hubs so that we can get bulk data
        master = hardwareMap.get(ExpansionHubEx.class, "master");
        slave = hardwareMap.get(ExpansionHubEx.class, "follower");


        // init imu
        imu = LynxOptimizedI2cFactory.createLynxEmbeddedImu(master.getStandardModule(), 0);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
        BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN);



        frontLeft = hardwareMap.get(ExpansionHubMotor.class, "frontLeft");
        backLeft = hardwareMap.get(ExpansionHubMotor.class, "backLeft");
        backRight = hardwareMap.get(ExpansionHubMotor.class, "backRight");
        frontRight = hardwareMap.get(ExpansionHubMotor.class, "frontRight");
        intakeLeft = hardwareMap.get(ExpansionHubMotor.class, "intakeLeft");
        intakeRight = hardwareMap.get(ExpansionHubMotor.class, "intakeRight");
        leftSlam = hardwareMap.get(ExpansionHubServo.class, "leftSlam");
        rightSlam = hardwareMap.get(ExpansionHubServo.class, "rightSlam");
        outtake = hardwareMap.get(ExpansionHubServo.class, "uttake");
        gripper = hardwareMap.get(ExpansionHubServo.class, "gripper");
        imu = hardwareMap.get(BNO055IMU.class, "imu");



        for (ExpansionHubMotor motor : driveMotors) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        for(ExpansionHubMotor expansionHubMotor : leftMotors) {
            expansionHubMotor.setDirection(DcMotor.Direction.REVERSE);
        }

        for(ExpansionHubMotor expansionHubMotor : intakeMotors) {
            expansionHubMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        intakeLeft.setDirection(DcMotor.Direction.REVERSE);


        setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDCoefficients(0,0,0));
    }

    @Override
    public PIDCoefficients getPIDCoefficients(DcMotor.RunMode runMode) {
        PIDFCoefficients coefficients = frontLeft.getPIDFCoefficients(runMode);
        return new PIDCoefficients(coefficients.p, coefficients.i, coefficients.d);
    }

    @Override
    public void setPIDCoefficients(DcMotor.RunMode runMode, PIDCoefficients coefficients) {
        for (ExpansionHubMotor motor : driveMotors) {
            motor.setPIDFCoefficients(runMode, new PIDFCoefficients(
                    coefficients.kP, coefficients.kI, coefficients.kD, 1
            ));
        }
    }



    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        RevBulkData bulkData = master.getBulkInputData();
        RevBulkData bulkData2 = slave.getBulkInputData();

        if (bulkData == null) {
            return Arrays.asList(0.0, 0.0, 0.0, 0.0);
        }

        List<Double> wheelPositions = new ArrayList<>();
    /*    for (ExpansionHubMotor motor : driveMotors) {
            wheelPositions.add(encoderTicksToInches(bulkData.getMotorCurrentPosition(motor)));
        }*/
        // TODO: !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! CHANGE THIS TO PORTN NUMBER
        wheelPositions.add(encoderTicksToInches(bulkData.getMotorCurrentPosition(1)));
        wheelPositions.add(encoderTicksToInches(bulkData.getMotorCurrentPosition(2)));
        wheelPositions.add(encoderTicksToInches(bulkData2.getMotorCurrentPosition(1)));
        wheelPositions.add(encoderTicksToInches(bulkData2.getMotorCurrentPosition(2)));
        return wheelPositions;
    }

    // TODO: ADD 2 BULK READS SINCE WE HAVE THE DRIVE MOTORS SPLIT UP

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        frontLeft.setPower(v);
        backLeft.setPower(v1);
        backRight.setPower(v2);
        frontRight.setPower(v3);
    }

    @Override
    public double getRawExternalHeading() {
        return imu.getAngularOrientation().firstAngle;
    }


    /**
     *
     * add other controls here
     */


    public boolean isIntakeBusy() {
        return intakeLeft.isBusy();
    }

    public void turnOnIntake() {
        intakeLeft.setPower(1);
        intakeRight.setPower(1);
    }

    public void turnOffIntake() {
        intakeRight.setPower(0);
        intakeLeft.setPower(0);
    }

    public void reverseIntake() {
        intakeLeft.setPower(-1);
        intakeRight.setPower(-1);
    }




    public void grabFoundation() {
        leftSlam.setPosition(leftDown);
        rightSlam.setPosition(rightDown);
    }

    public void reload() {
        leftSlam.setPosition(leftReload);
        rightSlam.setPosition(rightReload);
    }



    public boolean isOuttakeReady() {
        return outtake.getPosition() == outtakeReadyPosition;
    }

    public boolean isGripperReady() {
        return gripper.getPosition() == gripperOffPosition;
    }


    public void outtakeToOutPosition() {
        outtake.setPosition(outtakeOutPosition);
    }

    public void readyOuttake() {
        outtake.setPosition(outtakeReadyPosition);
    }

    public void grip() {
        gripper.setPosition(gripperOnPosition);
    }

    public void dontGrip() {
        gripper.setPosition(gripperOffPosition);
    }


}
