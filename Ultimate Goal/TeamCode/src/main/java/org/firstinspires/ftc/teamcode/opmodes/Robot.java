package org.firstinspires.ftc.teamcode.opmodes;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMUImpl;

import com.qualcomm.robotcore.hardware.HardwareMap;

import net.frogbots.ftcopmodetunercommon.opmode.TunableOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.teamcode.hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.movement.Odometry;
import org.firstinspires.ftc.teamcode.util.AxesSigns;
import org.firstinspires.ftc.teamcode.util.BNO055IMUUtil;
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.openftc.revextensions2.ExpansionHubMotor;



public class Robot extends TunableOpMode {

    public DriveTrain driveTrain;
//    public Intake intake;
//    public Shooter shooter;
//    public RingCirculatorThingIdk mover;

    // odom shit
    public Odometry odometry;
    private BNO055IMU imu;
    private double headingOffset;


    @Override
    public void init() {
        Hardware.loadParentOpMode(this);

        ExpansionHubMotor frontLeft, frontRight, backLeft, backRight;
        frontLeft = hardwareMap.get(ExpansionHubMotor.class, "FL");
        frontRight = hardwareMap.get(ExpansionHubMotor.class, "FR");
        backLeft = hardwareMap.get(ExpansionHubMotor.class, "BL");
        backRight = hardwareMap.get(ExpansionHubMotor.class, "BR");
        driveTrain = new DriveTrain(frontLeft, backLeft, frontRight, backRight);

        odometry = new Odometry(hardwareMap);
        initBNO055IMU(hardwareMap);

//        intake = new Intake(null, null);
//        shooter = new Shooter(null, null);

    }

    @Override
    public void init_loop() {
        telemetry.addLine(odometry.toString());
    }

    @Override
    public void start() {
//        odometrySet.markCurrOffset();
    }


    @Override
    public void loop() {
//        Hardware.updateAllHardwareComponents();
        driveTrain.update();
        updateOdometryComponents();
    }


    private void updateOdometryComponents() {
        double lastHeading = imu.getAngularOrientation().firstAngle - headingOffset;
        odometry.updateAngle(MathUtil.angleWrap(lastHeading + Odometry.startPosition.heading));
        odometry.update();
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
