package org.firstinspires.ftc.teamcode.control.system;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.control.path.Path;
import org.firstinspires.ftc.teamcode.util.Pose;

import java.util.LinkedList;

@Disabled
public abstract class BaseAuto extends Robot {
    public abstract LinkedList<Path> pathList();

    @Override
    public Pose startPose() {
        return new Pose(0,0,0);
    }

    @Override
    public void init() {
        super.init();
        setPathCache(pathList());
    }
}
