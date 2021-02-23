package org.firstinspires.ftc.teamcode.hardware;

import org.openftc.revextensions2.ExpansionHubServo;

public class WobbleGoal extends Hardware {
    private final double leftGrabberClosed = 0.6;
    private final double rightGrabberClosed = 0.25;
    private final double leftGrabberOpen = 0.25;
    private final double rightGrabberOpen = 0.6;
    private final double rightIn = 0.8;
    private final double rightOut = 0.3;
    private final double leftOut = 0.5;
    private final double leftIn = 0.0;
    private final ExpansionHubServo leftPivot;
    private final ExpansionHubServo rightPivot;
    private final ExpansionHubServo leftGrabber;
    private final ExpansionHubServo rightGrabber;

    public boolean isIn;
    public boolean isGrabbed;

    public WobbleGoal(ExpansionHubServo leftPivot, ExpansionHubServo rightPivot, ExpansionHubServo leftGrabber, ExpansionHubServo rightGrabber) {
        this.leftPivot = leftPivot;
        this.rightPivot = rightPivot;
        this.leftGrabber = leftGrabber;
        this.rightGrabber = rightGrabber;

        isIn = true;
        isGrabbed = true;
    }

    public void grab() {
        isGrabbed = true;
    }

    public void release() {
        isGrabbed = false;
    }

    public void out() {
        isIn = false;
    }

    public void in() {
        isIn = true;
    }

    // literally useless
    @Override
    public void turnOn() {

    }

    @Override
    public void turnOff() {

    }

    @Override
    public void reverse() {

    }

    @Override
    public void update() {
        if (isGrabbed && leftGrabber.getPosition() != leftGrabberClosed) {
            leftGrabber.setPosition(leftGrabberClosed);
            rightGrabber.setPosition(rightGrabberClosed);
        } else if (!isGrabbed && leftGrabber.getPosition() != leftGrabberOpen) {
            leftGrabber.setPosition(leftGrabberOpen);
            rightGrabber.setPosition(rightGrabberOpen);
        }

        if (isIn && leftPivot.getPosition() != leftIn) {
            leftPivot.setPosition(leftIn);
            rightPivot.setPosition(rightIn);
        } else if (!isIn && leftPivot.getPosition() != leftOut) {
            leftPivot.setPosition(leftOut);
            rightPivot.setPosition(rightOut);
        }
    }
}
