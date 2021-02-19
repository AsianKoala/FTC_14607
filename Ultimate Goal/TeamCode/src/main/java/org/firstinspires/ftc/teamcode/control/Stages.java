package org.firstinspires.ftc.teamcode.control;

public class Stages {
    public interface basicStage {
        String name();
        void startFunction();
        Results.baseResult mainFunction();
        void endFunction();
        StageStartVars AUTO_START_VARS = new StageStartVars();
    }
}
