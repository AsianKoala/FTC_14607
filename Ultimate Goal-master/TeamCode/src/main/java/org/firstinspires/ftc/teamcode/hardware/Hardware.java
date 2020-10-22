package org.firstinspires.ftc.teamcode.hardware;

import org.firstinspires.ftc.teamcode.auto.BaseOpMode;

public abstract class Hardware {
    protected static BaseOpMode parentOpMode;
    public static void loadBaseOpMode(BaseOpMode parent) { parentOpMode = parent; }

    public abstract void update();
    public abstract void debugUpdate();
}
