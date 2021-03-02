package org.firstinspires.ftc.teamcode.opmodes.auto;

import org.firstinspires.ftc.teamcode.control.controllers.Path;
import org.firstinspires.ftc.teamcode.opmodes.BaseOpMode;
import org.firstinspires.ftc.teamcode.util.Pose;

public abstract class BaseAuto extends BaseOpMode {

    public abstract Path path();

    @Override
    public Pose startPose() {
        return new Pose(0,0,0);
    }

    @Override
    public void loop() {
        robot.updateDashboardPath(path());
        super.loop();
        if(!robot.followPath(path()))
            stop();
    }
}
