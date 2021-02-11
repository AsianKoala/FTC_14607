package org.firstinspires.ftc.teamcode.opmodes;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMUImpl;

import com.qualcomm.robotcore.hardware.HardwareMap;

import net.frogbots.ftcopmodetunercommon.opmode.TunableOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.teamcode.hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.odometry.Odometry;
import org.firstinspires.ftc.teamcode.odometry.OdometrySet;
import org.firstinspires.ftc.teamcode.odometry.TwoWheelTrackingLocalizer;
import org.firstinspires.ftc.teamcode.util.AxesSigns;
import org.firstinspires.ftc.teamcode.util.BNO055IMUUtil;
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.Pose;
import org.openftc.revextensions2.ExpansionHubMotor;



public class Robot extends TunableOpMode {

    public DriveTrain driveTrain;
//    public Intake intake;
//    public Shooter shooter;
//    public RingCirculatorThingIdk mover;

    // odom shit
    public Odometry odometry;
    public OdometrySet odometrySet;
    private BNO055IMU imu;
    private double headingOffset;

    TwoWheelTrackingLocalizer localizer;

    @Override
    public void init() {
        Hardware.loadParentOpMode(this);

        ExpansionHubMotor frontLeft, frontRight, backLeft, backRight;
        frontLeft = hardwareMap.get(ExpansionHubMotor.class, "FL");
        frontRight = hardwareMap.get(ExpansionHubMotor.class, "FR");
        backLeft = hardwareMap.get(ExpansionHubMotor.class, "BL");
        backRight = hardwareMap.get(ExpansionHubMotor.class, "BR");
        driveTrain = new DriveTrain(frontLeft, backLeft, frontRight, backRight);

        ExpansionHubMotor verticalOdometer = hardwareMap.get(ExpansionHubMotor.class, "leftIntake");
        ExpansionHubMotor horizontalOdometer = hardwareMap.get(ExpansionHubMotor.class, "rightIntake");
//        odometrySet = new OdometrySet(verticalOdometer, horizontalOdometer);
//        odometry = new Odometry(new Pose(0, 0, Math.toRadians(90)), odometrySet);
        initBNO055IMU(hardwareMap);

//        intake = new Intake(null, null);
//        shooter = new Shooter(null, null);

        localizer = new TwoWheelTrackingLocalizer(hardwareMap);
        TwoWheelTrackingLocalizer.setStartHeading(Math.toRadians(0));
    }

    @Override
    public void init_loop() {
        telemetry.addLine(odometry.toString());
    }

    @Override
    public void start() {
        odometrySet.markCurrOffset();
    }


    @Override
    public void loop() {
//        Hardware.updateAllHardwareComponents();
        driveTrain.update();
//        updateOdometryComponents();
        double lastHeading = imu.getAngularOrientation().firstAngle - headingOffset;
        TwoWheelTrackingLocalizer.updateAngle(MathUtil.angleWrap(lastHeading + TwoWheelTrackingLocalizer.startHeading));
        telemetry.addLine(localizer.getPoseEstimate().toString());
    }


    private void updateOdometryComponents() {
        double lastHeading = imu.getAngularOrientation().firstAngle - headingOffset;
        odometry.update(MathUtil.angleWrap(lastHeading + Odometry.startHeading));
        telemetry.addLine(odometry.toString());

    }

    private void initBNO055IMU(HardwareMap hardwareMap) {
        imu = hardwareMap.get(BNO055IMUImpl.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.loggingEnabled  = false;
        imu.initialize(parameters);
        BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN);
        headingOffset = imu.getAngularOrientation().firstAngle;
    }

}
