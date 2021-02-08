package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.util.Range;

import org.openftc.revextensions2.ExpansionHubMotor;

public class RingCirculatorThingIdk extends Hardware {
    ExpansionHubMotor mover;
    public RingCirculatorThingIdk(ExpansionHubMotor mover) {
        this.mover = mover;

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
        mover.setPower(Range.clip(power, -1, 1));
    }
    @Override
    public void update() {

    }
}
