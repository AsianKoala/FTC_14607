package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.util.ArrayList;

public abstract class Hardware {
    public static OpMode parentOpMode;
    public static void loadParentOpMode(OpMode parentOpMod) {
        parentOpMode = parentOpMod;
    }
    public abstract void turnOn();
    public abstract void turnOff();
    public abstract void reverse();
    public abstract void update();
}
