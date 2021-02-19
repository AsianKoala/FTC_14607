package org.firstinspires.ftc.teamcode.control;

public class Stages {
    public interface basicStage {
        String name();
        boolean finishState();
        void startFunction();
        void mainFunction();
        void endFunction();
        StageStartVars AUTO_START_VARS = new StageStartVars();
    }

}
