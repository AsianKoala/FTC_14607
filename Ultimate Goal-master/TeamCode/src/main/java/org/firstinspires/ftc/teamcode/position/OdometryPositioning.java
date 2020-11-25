package org.firstinspires.ftc.teamcode.position;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.openftc.revextensions2.ExpansionHubMotor;


import static org.firstinspires.ftc.teamcode.util.Globals.*;
import static org.firstinspires.ftc.teamcode.util.UtilMethods.AngleWrap;

public class OdometryPositioning extends Hardware {

    private ExpansionHubMotor leftYEncoder, rightYEncoder, horizontalEncoder;
    private Rev2mDistanceSensor forwardDistanceSensor, rightDistanceSensor, backDistanceSensor, leftDistanceSensor;

    public double moveFactor = 12; // change to encoder factor scale thing
    public double turnFactor = 35; // change to encoder factor scale thing
    public double horizontalFactor = 12; // change to encoder factor scale thing
    public double horizontalPredictionFactor = 1; // experiemnt


    public double oldLeftPos = 0;
    public double oldRightPos = 0;
    public double oldHorizontalPos = 0;

    public double currLeftPos = 0;
    public double currRightPos = 0;
    public double currHorizontalPos = 0;


    // used for finding where we were at start of match
    public double leftInitialReading = 0;
    public double rightInitialReading = 0;
    public double lastResetAngle = 0;



    public OdometryPositioning(ExpansionHubMotor leftYAxisEncoder, ExpansionHubMotor rightYAxisEncoder, ExpansionHubMotor horizontalXEncoder,
                               Rev2mDistanceSensor forwardDistanceSensor, Rev2mDistanceSensor rightDistanceSensor, Rev2mDistanceSensor backDistanceSensor, Rev2mDistanceSensor leftDistanceSensor) {

        this.leftYEncoder = leftYAxisEncoder;
        this.rightYEncoder = rightYAxisEncoder;
        this.horizontalEncoder = horizontalXEncoder;

        this.forwardDistanceSensor = forwardDistanceSensor;
        this.rightDistanceSensor = rightDistanceSensor;
        this.backDistanceSensor = backDistanceSensor;
        this.leftDistanceSensor = leftDistanceSensor;


        leftYEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightYEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontalEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftYEncoder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightYEncoder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        horizontalEncoder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setPosition(double x, double y, double angle) {
        globalX = x;
        globalY = y;
        globalAngle = angle;

        leftInitialReading = currLeftPos;
        rightInitialReading = currRightPos;
        lastResetAngle = angle;
    }

    @Override
    public void update() {
        // update all current positions
        currLeftPos = leftYEncoder.getCurrentPosition();
        currRightPos = rightYEncoder.getCurrentPosition();
        currHorizontalPos = horizontalEncoder.getCurrentPosition();

        double leftCurr = -currLeftPos;
        double rightCurr = currRightPos;
        double horizontalCurr = currHorizontalPos;


        // difference in wheels
        double deltaLeft = leftCurr - oldLeftPos;
        double deltaRight = rightCurr - oldRightPos;
        double deltaHorizontal = horizontalCurr - oldHorizontalPos;


        //real units traveled
        double deltaLeftScaled = deltaLeft *  moveFactor;
        double deltaRightScaled = deltaRight * moveFactor;
        double deltaHorizontalScaled = deltaHorizontal * horizontalFactor;

        double deltaAngle = (deltaLeft - deltaRight) * turnFactor;

        // absolute calculation for worldAngle by comparing it to initial coords
        double leftTotal = -(currLeftPos - leftInitialReading);
        double rightTotal = currRightPos - rightInitialReading;
        globalAngle = AngleWrap(((leftTotal - rightTotal) * turnFactor) + lastResetAngle);



        // translation calculations
        // relative y translation
        double relativeY = (deltaLeftScaled + deltaRightScaled)/2;

        double horizontalPrediction = Math.toDegrees(deltaAngle) * horizontalPredictionFactor;
        double relativeX = deltaHorizontalScaled - horizontalPrediction;

        globalX += (Math.cos(globalAngle) * relativeY) + (Math.sin(globalAngle) * relativeX);
        globalY += (Math.sin(globalAngle) * relativeY) + (Math.cos(globalAngle) * relativeY);

        // use distance sensors to just subtract the readings from the starting coords to get current coords
        // if the distance sensors spike in readings switch back to odometry
        // use imu for angle reading
        // my idea is to use distance sensors only to reset our field coords every so often
        // so we arent fully relying on them for everything, just to make sure we're accurate
        // during auto we can reset our coords somwhere where we know it will be obstacle free
        // in teleop we can do something so once we need to auto align to fire, it will fix the coordinates
        // with the distance sensor and then use odo to rotate and fire or somethign like that


        oldLeftPos = currLeftPos;
        oldRightPos = currRightPos;
        oldHorizontalPos = currHorizontalPos;
    }

    @Override
    protected void debugUpdate() {

    }
}
