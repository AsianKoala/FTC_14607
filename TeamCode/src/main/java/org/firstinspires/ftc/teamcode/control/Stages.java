package org.firstinspires.ftc.teamcode.control;

public class Stages {
    interface BasicStage {
        AutoStartVars AUTO_START_VARS = new AutoStartVars();

        boolean finishState();

        Functions.function function();

        String name();
    }
}
