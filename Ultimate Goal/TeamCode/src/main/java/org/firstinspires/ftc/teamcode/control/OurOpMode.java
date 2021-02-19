package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.robotcore.hardware.HardwareMap;

import net.frogbots.ftcopmodetunercommon.opmode.TunableOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.movement.Odometry;
import org.firstinspires.ftc.teamcode.util.AxesSigns;
import org.firstinspires.ftc.teamcode.util.BNO055IMUUtil;
import org.firstinspires.ftc.teamcode.util.Pose;
import org.firstinspires.ftc.teamcode.util.Util;

public class OurOpMode extends TunableOpMode {
    public RobotHardware robot;
    // odom shit
    public Odometry odometry;
    private BNO055IMU imu;
    private double headingOffset;


    @Override
    public void init() {
        // load required instances
        Hardware.loadOpModeInstance(this);

        robot = new RobotHardware(hardwareMap);
        Functions.function.loadRobotInstance(robot);

        odometry = new Odometry(hardwareMap);
        odometry.setStart(new Pose(0, 0, 0));
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
        robot.update();
        updateOdometryComponents();
    }


    private void updateOdometryComponents() {
        double lastHeading = imu.getAngularOrientation().firstAngle - headingOffset;
        odometry.updateAngle(Util.angleWrap(lastHeading + Odometry.startPosition.heading));
        odometry.update();
        telemetry.addLine(odometry.toString());
    }

    private void initBNO055IMU(HardwareMap hardwareMap) {
        imu = hardwareMap.get(BNO055IMUImpl.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.loggingEnabled = false;
        imu.initialize(parameters);
        BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN);
        headingOffset = imu.getAngularOrientation().firstAngle;
    }
}
