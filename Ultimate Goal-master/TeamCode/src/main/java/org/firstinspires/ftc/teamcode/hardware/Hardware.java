package org.firstinspires.ftc.teamcode.hardware;

import org.firstinspires.ftc.teamcode.opmodes.BaseOpMode;

public abstract class Hardware {
    protected static BaseOpMode parentOpMode;
    public static void loadBaseOpMode(BaseOpMode parent) { parentOpMode = parent; }

    public abstract void update();
    protected abstract void debugUpdate();
}
