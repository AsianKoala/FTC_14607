package org.firstinspires.ftc.teamcode.opmodes.auto;

import org.firstinspires.ftc.teamcode.opmodes.BaseOpMode;
import org.firstinspires.ftc.teamcode.util.Pose;

public abstract class BaseAuto extends BaseOpMode {

    @Override
    public Pose startPose() {
        return new Pose(0,0,0);
    }

}
