package org.firstinspires.ftc.teamcode.hardware;

import org.openftc.revextensions2.ExpansionHubServo;

public class Actuator {
    public ExpansionHubServo actuator;
    public static final double pushVal = 0.55;
    public static final double pullVal = 0.175;
    public Actuator(ExpansionHubServo actuator) {
        this.actuator = actuator;
    }

    public void push() {
        actuator.setPosition(pushVal);
    }

    public void reset() {
        actuator.setPosition(pullVal);
    }
}
