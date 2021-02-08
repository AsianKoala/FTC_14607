package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.openftc.revextensions2.ExpansionHubMotor;

public class Intake extends Hardware {
    private final ExpansionHubMotor leftIntake, rightIntake;
    public Intake(ExpansionHubMotor leftIntake, ExpansionHubMotor rightIntake) {
        this.leftIntake = leftIntake;
        this.rightIntake = rightIntake;
        rightIntake.setDirection(DcMotorSimple.Direction.REVERSE);

        Hardware.allHardwareComponents.add(this);
    }

    public void turnOn() {
        setPower(1);
    }

    public void turnOff() {
        setPower(0);
    }

    public void reverse() {
        setPower(-1);
    }

    private void setPower(double power) {
        leftIntake.setPower(power);
        rightIntake.setPower(power);
    }

    @Override
    public void update() {

    }
}
