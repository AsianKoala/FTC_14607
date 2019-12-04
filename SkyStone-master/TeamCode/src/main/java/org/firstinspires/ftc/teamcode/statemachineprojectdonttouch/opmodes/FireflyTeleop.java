package org.firstinspires.ftc.teamcode.statemachineprojectdonttouch.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.statemachineprojectdonttouch.HelperClasses.Firefly;
import org.firstinspires.ftc.teamcode.statemachineprojectdonttouch.HelperClasses.TimeProfiler;

@TeleOp(name = "new test teleop", group = "teleop")
public class FireflyTeleop extends Firefly {


    @Override
    public void init() {
        super.init();
    }

    @Override
    public void init_loop() {
        super.init_loop();
    }

    @Override
    public void start() {
        super.start();
    }


    private TimeProfiler tp1 = new TimeProfiler(1000);


    @Override
    public void loop() {


        tp1.markStart();
        super.loop();
        tp1.markEnd();

        telemetry.addLine("-------------- FIREFLY TELEOP TELEMETRY -----------------");
        telemetry.addLine("tOp profiler 1: " + tp1.getAverageTimePerUpdateMillis());


    }





}