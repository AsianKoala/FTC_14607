package org.firstinspires.ftc.teamcode.control;


import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

import java.util.ArrayList;

public class Functions {
    public static abstract class function {
        public static RobotHardware robot;

        public static void loadRobotInstance(RobotHardware robot) {
            function.robot = robot;
        }
    }

    public static abstract class simpleFunction extends function {
        public abstract void method();
    }

    public static abstract class loopFunction extends function {
        public abstract Results.simpleResult method();
    }

    public static abstract class conditionalFunction extends function {
        public abstract boolean startCondition();

        public abstract Results.simpleResult method();
    }

    public static abstract class listFunction extends function {
        public abstract ArrayList<conditionalFunction> allFunctions();
    }


}


