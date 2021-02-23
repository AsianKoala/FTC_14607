package org.firstinspires.ftc.teamcode.hardware;

import org.openftc.revextensions2.ExpansionHubServo;

public class WobbleGoal {
    private final double leftGrabberClosed = 0.6;
    private final double rightGrabberClosed = 0.25;
    private final double leftGrabberOpen = 0.25;
    private final double rightGrabberOpen = 0.6;
    private final double rightIn = 0.8;
    private final double rightOut = 0.3;
    private final double leftOut = 0.5;
    private final double leftIn = 0.0;
    private ExpansionHubServo leftPivot, rightPivot, leftGrabber, rightGrabber;

    public WobbleGoal(ExpansionHubServo leftPivot, ExpansionHubServo rightPivot, ExpansionHubServo leftGrabber, ExpansionHubServo rightGrabber) {
        this.leftPivot = leftPivot;
        this.rightPivot = rightPivot;
        this.leftGrabber = leftGrabber;
        this.rightGrabber = rightGrabber;
    }

    public void grab() {
        leftGrabber.setPosition(leftGrabberClosed);
        rightGrabber.setPosition(rightGrabberClosed);
    }

    public void release() {
        leftGrabber.setPosition(leftGrabberOpen);
        rightGrabber.setPosition(rightGrabberOpen);
    }

    public void out() {
        leftPivot.setPosition(leftOut);
        rightPivot.setPosition(rightOut);
    }

    public void in() {
        leftPivot.setPosition(leftIn);
        rightPivot.setPosition(rightIn);
    }
}
