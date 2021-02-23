package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public abstract class Hardware {
    public static OpMode opmodeInstance; // this is so stupid LMAO
    public static void loadOpModeInstance(OpMode parentOpMod) {
        opmodeInstance = parentOpMod;
    }

    public abstract void turnOn();
    public abstract void turnOff();
    public abstract void reverse();
    public abstract void update();
}