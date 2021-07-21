package org.firstinspires.ftc.teamcode.control.system;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.control.path.Path;

@Disabled
public abstract class BaseAuto extends Robot {
    public abstract Path path();

    @Override
    public void init() {
        super.init();
        setPathCache(path());
    }
}
