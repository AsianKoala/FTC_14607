package org.firstinspires.ftc.teamcode.control.localization;

public class EncoderWheel {
    public double x, y, heading;
    public int row; // Row in matrix

    EncoderWheel(double x, double y, double heading, int row) {
        this.x = x;
        this.y = y;
        this.heading = heading;
        this.row = row;
    }
}