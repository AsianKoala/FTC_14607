package org.firstinspires.ftc.teamcode.statemachineprojectdonttouch.HelperClasses;

public class OdometryModule {
    double currReading;
    public OdometryModule() {
        currReading = 0;
    }

    public void setReading(double reading) {
        currReading = reading;
    }

    public double getReading() {
        return currReading;
    }
}