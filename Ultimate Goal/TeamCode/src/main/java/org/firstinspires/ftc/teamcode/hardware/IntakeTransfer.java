package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.util.Range;

import org.openftc.revextensions2.ExpansionHubMotor;

public class IntakeTransfer extends Hardware {
    private final ExpansionHubMotor mover;
    private double moverPower;
    public IntakeTransfer(ExpansionHubMotor mover) {
        this.mover = mover;
        moverPower = 0;
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
        moverPower = power;
    }

    @Override
    public void update() {
        mover.setPower(moverPower);
    }
}
