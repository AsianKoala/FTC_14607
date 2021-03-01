package org.firstinspires.ftc.teamcode.opmodes;

import org.firstinspires.ftc.teamcode.util.Pose;

public abstract class BaseAuto extends BaseOpMode {

    @Override
    public Pose startPose() {
        return new Pose(0,0,0); // todo
    }

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

    @Override
    public void loop() {
        super.loop();
    }
}
