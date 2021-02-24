package org.firstinspires.ftc.teamcode.main.control;

public class Stages {
    public interface basicStage {
        StageStartVars AUTO_START_VARS = new StageStartVars();
        String name();
        void startFunction();
        Results.baseResult mainFunction();
        void endFunction();
    }
}
