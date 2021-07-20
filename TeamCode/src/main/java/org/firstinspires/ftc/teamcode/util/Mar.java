package org.firstinspires.ftc.teamcode.util;

public class Mar {
    //@TODO lol fix this shit
    private double time;
    private double start;

    public Mar() {
        time = 0;
        start = 0;
    }
    public void start() {
        start = System.currentTimeMillis();
    }

    public void stop() {
        time = System.currentTimeMillis() - start;
    }

    public double getTime() {
        return time;
    }
}
