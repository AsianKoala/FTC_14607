package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.util.OpModeClock;

public class BaseOpMode extends OpMode {

    @Override
    public void init() {
        OpModeClock.markInit();
    }

    @Override
    public void start() {
        super.start();
        OpModeClock.markStart();
    }

    @Override
    public void loop() {

    }
}
