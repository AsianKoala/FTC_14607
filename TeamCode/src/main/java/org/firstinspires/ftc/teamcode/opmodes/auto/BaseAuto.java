package org.firstinspires.ftc.teamcode.opmodes.auto;

import org.firstinspires.ftc.teamcode.opmodes.Robot;
import org.firstinspires.ftc.teamcode.util.Pose;

public abstract class BaseAuto extends Robot {

    @Override
    public Pose startPose() {
        return new Pose(0,0,0);
    }

}
