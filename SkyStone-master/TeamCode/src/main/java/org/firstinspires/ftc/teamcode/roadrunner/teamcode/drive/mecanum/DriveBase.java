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

public class DriveBase extends SampleMecanumDriveBase {
    private ExpansionHubEx master, slave;
    private ExpansionHubMotor frontLeft, backLeft, backRight, frontRight;
    private ArrayList<ExpansionHubMotor> driveMotors = new ArrayList<>();
    private ArrayList<ExpansionHubMotor> leftMotors = new ArrayList<>();

    private BNO055IMU imu;

    public DriveBase(HardwareMap hardwareMap) {
        super();

        driveMotors.add(frontLeft);
        driveMotors.add(frontRight);
        driveMotors.add(backLeft);
        driveMotors.add(backRight);
        leftMotors.add(frontLeft);
        leftMotors.add(backLeft);


        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);


        master = hardwareMap.get(ExpansionHubEx.class, "master");
        slave = hardwareMap.get(ExpansionHubEx.class, "follower");



        imu = LynxOptimizedI2cFactory.createLynxEmbeddedImu(master.getStandardModule(), 0);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN);





        frontLeft = hardwareMap.get(ExpansionHubMotor.class, "frontLeft");
        backLeft = hardwareMap.get(ExpansionHubMotor.class, "backLeft");
        backRight = hardwareMap.get(ExpansionHubMotor.class, "backRight");
        frontRight = hardwareMap.get(ExpansionHubMotor.class, "frontRight");



        for (ExpansionHubMotor motor : driveMotors) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        for (ExpansionHubMotor motor: otherMotors) {
            // set intakes to without encoders for speed
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        for(ExpansionHubMotor expansionHubMotor : leftMotors) {
            expansionHubMotor.setDirection(DcMotor.Direction.REVERSE);
        }

        // REVERSE LEFT INTKAE
        leftIntake.setDirection(DcMotor.Direction.REVERSE);



        myIntake = new Intake(this, leftIntake, rightIntake);
        myIntake.turnOffIntake();

        setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDCoefficients(0,0,0));
    }
    public void intakeOn() {
        myIntake.turnOnIntake();
    }

    public void intakeOff() {
        myIntake.turnOffIntake();
    }

    public void intakeReversed() {
        myIntake.turnOnReverseIntake();
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



   // @NonNull
  //  @Override
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
}
