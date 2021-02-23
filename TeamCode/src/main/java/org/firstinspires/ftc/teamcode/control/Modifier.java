package org.firstinspires.ftc.teamcode.control;


import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

public class Modifier {
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


