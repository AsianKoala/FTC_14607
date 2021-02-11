package org.firstinspires.ftc.teamcode.odometry;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.openftc.revextensions2.ExpansionHubMotor;

public class OdometrySet {
    private final ExpansionHubMotor vertical, horizontal;
    private int verticalOffset = 0;
    private int horizontalOffset = 0;

    public OdometrySet(ExpansionHubMotor vertical, ExpansionHubMotor horizontal) {
        this.vertical = vertical;
        this.horizontal = horizontal;
        vertical.setDirection(DcMotorSimple.Direction.REVERSE);
        horizontal.setDirection(DcMotorSimple.Direction.REVERSE);
        markCurrOffset();
    }

    public void markCurrOffset() {
        verticalOffset = vertical.getCurrentPosition();
        horizontalOffset = horizontal.getCurrentPosition();
    }

    public int getVerticalTicks() {
        return vertical.getCurrentPosition() - verticalOffset;
    }

    public int getHorizontalTicks() {
        return horizontal.getCurrentPosition() - horizontalOffset;
    }
}
