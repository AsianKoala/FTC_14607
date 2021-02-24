package org.firstinspires.ftc.teamcode.main.control;


import org.firstinspires.ftc.teamcode.main.hardware.RobotHardware;

public class Modifiers {
    public static abstract class modifier {
        public static RobotHardware robot;
        public static void loadRobotInstance(RobotHardware robot) {
            modifier.robot = robot;
        }


    }

    public static abstract class turnModifier extends modifier {
        public abstract double angle();
        public abstract Results.turnResult turnMethod();
    }

    public static abstract class hardwareModifier extends modifier {
        public abstract Results.diffResult hardwareMethod();
    }


}


