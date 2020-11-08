package org.firstinspires.ftc.teamcode.position;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.openftc.revextensions2.ExpansionHubMotor;

public class OdometryPositioning extends Hardware {

    private ExpansionHubMotor leftYEncoder, rightYEncoder, horizontalEncoder;
    private double oldLeftPosition, oldRightPosition, oldHorizontalPosition = 0;
    private double leftPosition, rightPosition, horizontalPosition = 0;
    private double distanceBetweenWheels = 15; // change this later


    public OdometryPositioning(ExpansionHubMotor leftYAxisEncoder, ExpansionHubMotor rightYAxisEncoder, ExpansionHubMotor horizontalXEncoder) {
        this.leftYEncoder = leftYAxisEncoder;
        this.rightYEncoder = rightYAxisEncoder;
        this.horizontalEncoder = horizontalXEncoder;

        leftYEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightYEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontalEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftYEncoder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightYEncoder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        horizontalEncoder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightYEncoder.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setPositions(double currLeftPosition, double currRightPosition, double currHorizontal) {
        leftPosition = currLeftPosition;
        rightPosition = currRightPosition;
        horizontalPosition = currHorizontal;
        oldLeftPosition = leftPosition;
        oldRightPosition = rightPosition;
        oldHorizontalPosition = horizontalPosition;
    }

    public void getPositions(double currLeftPosition, double currRightPosition, double currHorizontal) {
        leftPosition = currLeftPosition;
        rightPosition = currRightPosition;
        horizontalPosition = currHorizontal;
    }

    @Override
    public void update() {
        double deltaLeftScale, deltaRightScale, deltaHorizontalScale;
        deltaLeftScale = (leftPosition - oldLeftPosition);
        deltaRightScale = rightPosition - oldRightPosition;
        deltaHorizontalScale = horizontalPosition - oldHorizontalPosition;
    }

    @Override
    protected void debugUpdate() {

    }
}
