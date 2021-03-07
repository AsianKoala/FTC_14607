package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;

import net.frogbots.ftcopmodetunercommon.opmode.TunableOpMode;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.util.OpModeClock;
import org.firstinspires.ftc.teamcode.util.Pose;

public abstract class BaseOpMode extends TunableOpMode {

    public abstract Pose startPose();
    public Robot robot;

    @Override
    public void init() {
        OpModeClock.markInit();
        robot = new Robot(startPose(), hardwareMap, FtcDashboard.getInstance());

    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        OpModeClock.markStart();
    }

    @Override
    public void loop() {
        robot.update();
    }
}
