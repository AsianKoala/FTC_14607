package org.firstinspires.ftc.teamcode.opmodes;

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

    @Override
    public void init() {
        // used for each subsystem to send telemetry
        Hardware.loadBaseOpMode(this);

        leftFront = hardwareMap.get(ExpansionHubMotor.class, "leftFront");
        leftBack = hardwareMap.get(ExpansionHubMotor.class, "leftBack");
        rightFront = hardwareMap.get(ExpansionHubMotor.class, "rightFront");
        rightBack = hardwareMap.get(ExpansionHubMotor.class, "rightBack");
        driveTrain = new DriveTrain(leftFront, leftBack, rightFront, rightBack);

        leftYAxisEncoder = hardwareMap.get(ExpansionHubMotor.class, "leftXAxisEncoder");
        rightYAxisEncoder = hardwareMap.get(ExpansionHubMotor.class, "rightXAxisEncoder");
        horizontalXEncoder = hardwareMap.get(ExpansionHubMotor.class, "horizontalYAxisEncoder");
        odometryPositioning = new OdometryPositioning(leftYAxisEncoder, rightYAxisEncoder, horizontalXEncoder);
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
    }
}
