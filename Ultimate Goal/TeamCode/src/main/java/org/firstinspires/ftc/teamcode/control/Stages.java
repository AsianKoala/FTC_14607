package org.firstinspires.ftc.teamcode.control;

public class Stages {
    interface BasicStage {
        boolean finishState();
        void stage();
        StageVars stageVars = new StageVars();
        String name();
    }
}
