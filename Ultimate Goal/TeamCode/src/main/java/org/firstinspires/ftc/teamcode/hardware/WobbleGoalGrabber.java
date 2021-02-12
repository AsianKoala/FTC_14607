package org.firstinspires.ftc.teamcode.hardware;

import org.openftc.revextensions2.ExpansionHubServo;

public class WobbleGoalGrabber {
    public ExpansionHubServo grabber;
    public static double grabPosition = 0;
    public static double releasePosition = 1;
    public WobbleGoalGrabber(ExpansionHubServo grabberServo) {
        this.grabber = grabberServo;
    }

    public void grab() {
        grabber.setPosition(grabPosition);
    }

    public void release() {
        grabber.setPosition(releasePosition);
    }
}
