package org.firstinspires.ftc.teamcode.control;

public class Stages {
    interface BasicStage {
        boolean finishState();
        Functions.function function();
        AutoStartVars AUTO_START_VARS = new AutoStartVars();
        String name();
    }
}
