package org.firstinspires.ftc.teamcode.control.localization;

import org.openftc.revextensions2.ExpansionHubMotor;

public class OdometrySet {
    private ExpansionHubMotor vertical, horizontal;
    private int verticalOffset = 0;
    private int horizontalOffset = 0;

    public OdometrySet(ExpansionHubMotor vertical, ExpansionHubMotor horizontal) {
        this.vertical = vertical;
        this.horizontal = horizontal;
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