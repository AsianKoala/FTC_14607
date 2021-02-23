package org.firstinspires.ftc.teamcode.hardware;

import org.openftc.revextensions2.ExpansionHubServo;

public class Actuator extends Hardware {
    public static final double pushVal = 0.55;
    public static final double pullVal = 0.175;
    public double currentVal = pullVal;
    public ExpansionHubServo actuator;

    public Actuator(ExpansionHubServo actuator) {
        this.actuator = actuator;
    }

    @Override
    public void turnOn() {
        currentVal = pushVal;
    }

    @Override
    public void turnOff() {
// idk wtf xd
    }

    @Override
    public void reverse() {
        currentVal = pullVal;
    }

    @Override
    public void update() {
        if (!(actuator.getPosition() == currentVal)) { // for my mental health lmao
            actuator.setPosition(currentVal);
        }
    }
}
