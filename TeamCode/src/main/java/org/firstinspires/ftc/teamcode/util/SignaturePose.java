package org.firstinspires.ftc.teamcode.util;

public class SignaturePose extends Pose {
    public long sign;
    public SignaturePose(double x, double y, double heading, long sign) {
        super(x,y,heading);
        this.sign = sign;
    }

    public SignaturePose(Pose p, long sign) {
        super(p);
        this.sign = sign;
    }

    public SignaturePose(Pose p) {
        super(p);
        this.sign = System.currentTimeMillis();
    }
}
