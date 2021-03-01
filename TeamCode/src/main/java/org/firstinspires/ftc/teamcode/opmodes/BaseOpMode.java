package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import net.frogbots.ftcopmodetunercommon.opmode.TunableOpMode;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.util.OpModeClock;
import org.firstinspires.ftc.teamcode.util.Point;
import org.firstinspires.ftc.teamcode.util.Pose;

import java.util.ArrayList;

public abstract class BaseOpMode extends TunableOpMode {

    // public abstract StateMachine stateMachine;
    public abstract Pose startPose();
    public abstract ArrayList<Point> allPoints();

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
        robot.updateDashboard(allPoints());
    }
}
