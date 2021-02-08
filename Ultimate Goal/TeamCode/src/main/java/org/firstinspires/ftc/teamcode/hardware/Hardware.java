package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.util.ArrayList;

public abstract class Hardware {
    protected static OpMode parentOpMode;
    public static ArrayList<Hardware> allHardwareComponents;
    public static void loadParentOpMode(OpMode parentOpMod) {
        parentOpMode = parentOpMod;
    }

    protected abstract void update();

    public static void updateAllHardwareComponents() {
        for(Hardware component : allHardwareComponents) {
            component.update();
        }
    }
}
