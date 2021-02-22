package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.robotcore.hardware.HardwareMap;

import net.frogbots.ftcopmodetunercommon.opmode.TunableOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.teamcode.hardware.Actuator;
import org.firstinspires.ftc.teamcode.hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.hardware.Shooter;
import org.firstinspires.ftc.teamcode.hardware.WobbleGoal;
import org.firstinspires.ftc.teamcode.movement.Odometry;
import org.firstinspires.ftc.teamcode.util.AxesSigns;
import org.firstinspires.ftc.teamcode.util.BNO055IMUUtil;
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.Pose;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.ExpansionHubServo;

public class Robot extends TunableOpMode {
    public DriveTrain driveTrain;
    public Shooter shooter;
    public Actuator actuator;
    public WobbleGoal wobbleGoal;

    // odom shit
    public Odometry odometry;
    private BNO055IMU imu;
    private double headingOffset;


    @Override
    public void init() {
        Hardware.loadParentOpMode(this);

        ExpansionHubMotor frontLeft, frontRight, backLeft, backRight, launcher1, launcher2;
        ExpansionHubServo actuatorServo, leftPivot, rightPivot, leftGrabber, rightGrabber;
        frontLeft = hardwareMap.get(ExpansionHubMotor.class, "FL");
        frontRight = hardwareMap.get(ExpansionHubMotor.class, "FR");
        backLeft = hardwareMap.get(ExpansionHubMotor.class, "BL");
        backRight = hardwareMap.get(ExpansionHubMotor.class, "BR");
        launcher1 = hardwareMap.get(ExpansionHubMotor.class, "launcher1");
        launcher2 = hardwareMap.get(ExpansionHubMotor.class, "launcher2");
        actuatorServo = hardwareMap.get(ExpansionHubServo.class, "actuator");
        leftPivot = hardwareMap.get(ExpansionHubServo.class, "leftPivot");
        rightPivot = hardwareMap.get(ExpansionHubServo.class, "rightPivot");
        leftGrabber = hardwareMap.get(ExpansionHubServo.class, "leftGrabber");
        rightGrabber = hardwareMap.get(ExpansionHubServo.class, "rightGrabber");

        driveTrain = new DriveTrain(frontLeft, frontRight, backLeft, backRight);
        shooter = new Shooter(launcher1, launcher2);
        actuator = new Actuator(actuatorServo);
        wobbleGoal = new WobbleGoal(leftPivot, rightPivot, leftGrabber, rightGrabber);
        odometry = new Odometry(hardwareMap);
        odometry.setStart(new Pose(0,0,0));
        initBNO055IMU(hardwareMap);
    }

    @Override
    public void init_loop() {
        super.init_loop();
        telemetry.addLine(odometry.toString());
    }

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void loop() {
        telemetry.clear();
        driveTrain.update();
        shooter.update();
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
