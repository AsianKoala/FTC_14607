package org.firstinspires.ftc.teamcode.code;

import android.support.annotation.NonNull;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.teamcode.roadrunner.teamcode.drive.mecanum.SampleMecanumDriveBase;
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
import static org.firstinspires.ftc.teamcode.code.GLOBALCONSTANTS.*;
import static org.firstinspires.ftc.teamcode.roadrunner.teamcode.drive.DriveConstants.encoderTicksToInches;

/*
 * Optimized mecanum drive implementation for REV ExHs. The time savings may significantly improve
 * trajectory following performance with moderate additional complexity.
 */

public class HouseFly extends SampleMecanumDriveBase {
    public ExpansionHubEx master, slave;
    public ExpansionHubMotor frontLeft, backLeft, backRight, frontRight;
    public ExpansionHubMotor leftIntake, rightIntake;


    public ExpansionHubMotor leftSlide, rightSlide; // tfw when you dont have to worry about this bs cause charlie is coding teleop edit: rip
    public ExpansionHubServo leftSlam, rightSlam;
    public ExpansionHubServo rotater;
    public ExpansionHubServo gripper;
    public ExpansionHubServo flipper;



    private ArrayList<ExpansionHubMotor> driveMotors = new ArrayList<>();
    private ArrayList<ExpansionHubMotor> leftMotors = new ArrayList<>();
    private ArrayList<ExpansionHubMotor> rightMotors = new ArrayList<>();
    private ArrayList<ExpansionHubMotor> intakeMotors = new ArrayList<>();
    private ArrayList<ExpansionHubMotor> slideMotors = new ArrayList<>();

    private BNO055IMU imu;


    /**
     * variable declaration
     */






    public HouseFly(HardwareMap hardwareMap) {
        super();



    //    LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);


        // map hubs so that we can get bulk data
        master = hardwareMap.get(ExpansionHubEx.class, "master");
        slave = hardwareMap.get(ExpansionHubEx.class, "follower");


        // init imu
        imu = LynxOptimizedI2cFactory.createLynxEmbeddedImu(master.getStandardModule(), 0);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
        BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN);



        frontLeft = hardwareMap.get(ExpansionHubMotor.class, "FL");
        backLeft = hardwareMap.get(ExpansionHubMotor.class, "BL");
        backRight = hardwareMap.get(ExpansionHubMotor.class, "BR");
        frontRight = hardwareMap.get(ExpansionHubMotor.class, "FR");
        leftIntake = hardwareMap.get(ExpansionHubMotor.class, "leftIntake");
        rightIntake = hardwareMap.get(ExpansionHubMotor.class, "rightIntake");
        leftSlide = hardwareMap.get(ExpansionHubMotor.class, "leftSlide");
        rightSlide = hardwareMap.get(ExpansionHubMotor.class, "rightSlide");
        leftSlam = hardwareMap.get(ExpansionHubServo.class, "leftSlam");
        rightSlam = hardwareMap.get(ExpansionHubServo.class, "rightSlam");
        rotater = hardwareMap.get(ExpansionHubServo.class, "rotater");
        gripper = hardwareMap.get(ExpansionHubServo.class, "gripper");
        flipper = hardwareMap.get(ExpansionHubServo.class, "flipper");
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


        // TODO: charlie change this to what you want
        for(ExpansionHubMotor expansionHubMotor : slideMotors) {
            expansionHubMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //expansionHubMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        leftIntake.setDirection(DcMotor.Direction.REVERSE);
        leftSlide.setDirection(DcMotor.Direction.REVERSE);


        // TODO: edit this value to tuned PD
        setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDCoefficients(0,0,0));


        driveMotors.add(frontLeft);
        driveMotors.add(frontRight);
        driveMotors.add(backLeft);
        driveMotors.add(backRight);
        leftMotors.add(frontLeft);
        leftMotors.add(backLeft);
        rightMotors.add(frontRight);
        rightMotors.add(backRight);
        intakeMotors.add(leftIntake);
        intakeMotors.add(rightIntake);
        slideMotors.add(leftSlide);
        slideMotors.add(rightSlide);
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


        wheelPositions.add(encoderTicksToInches(bulkData.getMotorCurrentPosition(0)));
        wheelPositions.add(encoderTicksToInches(bulkData.getMotorCurrentPosition(1)));
        wheelPositions.add(encoderTicksToInches(bulkData2.getMotorCurrentPosition(0)));
        wheelPositions.add(encoderTicksToInches(bulkData2.getMotorCurrentPosition(1)));
        return wheelPositions;
    }
    


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
    






    public double getWrappedHeading() {
        return AngleWrap(imu.getAngularOrientation().firstAngle);
    }

    
    public double AngleWrap(double rad) {
        while(rad > Math.PI * 2) {
            rad -= Math.PI * 2;
        }
        
        return rad;
    }

    public boolean intakeBusy() {
        return leftIntake.isBusy() || rightIntake.isBusy();}

    public void setIntakePowers(double leftIntakePower, double rightIntakePower) {
        leftIntake.setPower(leftIntakePower);
        rightIntake.setPower(rightIntakePower);
    }


    public void stopIntake() { setIntakePowers(0,0);}


    /**
     * foundation movement controls
     *
     *
     */

    public void grabFoundation() {
        leftSlam.setPosition(0.9);
        rightSlam.setPosition(0.1);
    }

    public void ungrabFoundation() {
        leftSlam.setPosition(0.1);
        rightSlam.setPosition(0.9);
    }


    /**
     *
     *
     * flipper movement controls
     */

    public void flipOut() {
        flipper.setPosition(flipperOut);
    }

    public void flipHome() {
        flipper.setPosition(flipperHome);
    }

    public void flipBetween() {
        flipper.setPosition(flipperBetween);
    }

    public void flipBetweenBetween() {
        flipper.setPosition(flipperBetweenBetween);
    }


    /**
     * gripper controls
     */

    public void grip() {
        gripper.setPosition(gripperGrip);
    }

    public void gripHome() {
        gripper.setPosition(gripperHome);
    }


    /**
     * rotater movement controls
     */

    public void rotaterOut() {
        rotater.setPosition(rotaterOut);
    }

    public void rotaterHome() {
        rotater.setPosition(rotaterHome);
    }


}
