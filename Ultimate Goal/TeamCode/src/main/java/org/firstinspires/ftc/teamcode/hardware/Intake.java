package org.firstinspires.ftc.teamcode.hardware;

import org.openftc.revextensions2.ExpansionHubMotor;

public class Intake extends Hardware {
    private final ExpansionHubMotor intake;
    private double intakePower;
    public Intake(ExpansionHubMotor intake) {
        this.intake = intake;
        intakePower = 0;
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
        intakePower = power;
    }

    @Override
    public void update() {
        intake.setPower(intakePower);
    }
}
