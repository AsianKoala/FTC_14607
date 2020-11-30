package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;

import net.frogbots.ftcopmodetunercommon.opmode.TunableOpMode;

import org.firstinspires.ftc.teamcode.hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.position.OdometryPositioning;
import org.openftc.revextensions2.ExpansionHubMotor;


public class BaseOpMode extends TunableOpMode {

    private DriveTrain driveTrain;
    private ExpansionHubMotor leftFront, leftBack, rightFront, rightBack;

    private OdometryPositioning odometryPositioning;
    private ExpansionHubMotor leftYAxisEncoder, rightYAxisEncoder, horizontalXEncoder;
    private Rev2mDistanceSensor forwardDistanceSensor, leftDistanceSensor, rightDistanceSensor, backDistanceSensor;

    private BNO055IMU imu;

    @Override
    public void init() {
        // used for each subsystem to send telemetry
        Hardware.loadBaseOpMode(this);

        leftFront = hardwareMap.get(ExpansionHubMotor.class, "leftFront");
        leftBack = hardwareMap.get(ExpansionHubMotor.class, "leftBack");
        rightFront = hardwareMap.get(ExpansionHubMotor.class, "rightFront");
        rightBack = hardwareMap.get(ExpansionHubMotor.class, "rightBack");
        driveTrain = new DriveTrain(leftFront, leftBack, rightFront, rightBack);


        // odom
        leftYAxisEncoder = hardwareMap.get(ExpansionHubMotor.class, "leftXAxisEncoder");
        rightYAxisEncoder = hardwareMap.get(ExpansionHubMotor.class, "rightXAxisEncoder");
        horizontalXEncoder = hardwareMap.get(ExpansionHubMotor.class, "horizontalYAxisEncoder");
        forwardDistanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "fwdDistance");
        rightDistanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "rightDistance");
        backDistanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "backDistance");
        leftDistanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "leftDistance");
        odometryPositioning = new OdometryPositioning(leftYAxisEncoder, rightYAxisEncoder, horizontalXEncoder, forwardDistanceSensor, rightDistanceSensor, backDistanceSensor, leftDistanceSensor);

        // imu
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);


        // for debugging set init position at x=0, y=0, heading=0
        odometryPositioning.setPosition(0,0,0);
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {

        driveTrain.update();

        odometryPositioning.update();
    }
}
