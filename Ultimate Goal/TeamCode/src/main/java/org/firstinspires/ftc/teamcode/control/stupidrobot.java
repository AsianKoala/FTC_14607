package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.IntakeTransfer;
import org.openftc.revextensions2.ExpansionHubMotor;

public class stupidrobot extends OpMode {
    public DriveTrain driveTrain;
    public Intake intake;
    public IntakeTransfer intakeTransfer;

    @Override
    public void init() {
        ExpansionHubMotor frontLeft, frontRight, backLeft, backRight, intakeMotor, intakeTransferMotor;
        frontLeft = hardwareMap.get(ExpansionHubMotor.class, "FL");
        frontRight = hardwareMap.get(ExpansionHubMotor.class, "FR");
        backLeft = hardwareMap.get(ExpansionHubMotor.class, "BL");
        backRight = hardwareMap.get(ExpansionHubMotor.class, "BR");
        intakeMotor = hardwareMap.get(ExpansionHubMotor.class, "intake");
        intakeTransferMotor = hardwareMap.get(ExpansionHubMotor.class, "intakeTransfer");
        driveTrain = new DriveTrain(frontLeft, frontRight, backLeft, backRight);
        intake = new Intake(intakeMotor);
        intakeTransfer = new IntakeTransfer(intakeTransferMotor);
    }

    @Override
    public void loop() {
        driveTrain.update();
        intake.update();
        intakeTransfer.update();
    }
}
