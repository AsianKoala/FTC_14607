package org.firstinspires.ftc.teamcode.main.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.ExpansionHubServo;

import java.util.ArrayList;

public class RobotHardware {
    public final boolean isDebugging = false;
    public DriveTrain driveTrain;
    public Actuator actuator;
    public Shooter shooter;
    public WobbleGoal wobbleGoal;
    public ArrayList<Hardware> allHardware;

    public RobotHardware(HardwareMap hardwareMap) {
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

        allHardware = new ArrayList<>();
        allHardware.add(driveTrain);
        allHardware.add(shooter);
        allHardware.add(actuator);
        allHardware.add(wobbleGoal);
    }

    public void update() {
        for (Hardware h : allHardware) {
            h.update();

            if (isDebugging)
                Hardware.opmodeInstance.telemetry.addLine(h.toString());
        }
    }
}
