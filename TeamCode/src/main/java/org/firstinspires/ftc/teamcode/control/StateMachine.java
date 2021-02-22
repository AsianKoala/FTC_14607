package org.firstinspires.ftc.teamcode.control;

import java.util.LinkedList;
import static org.firstinspires.ftc.teamcode.control.Stages.*;


public class StateMachine {
    public LinkedList<BasicStage> stages;
    public BasicStage currStage;

    public StateMachine() {
        stages = new LinkedList<>();
    }

    public void addStage(BasicStage newStage) {
        stages.add(newStage);
        currStage = stages.getFirst();
    }

    public void loop() {
        if(currStage.finishState()) {
            stages.removeFirst();
            if(!done()) {
                currStage = stages.getFirst();
                currStage.AUTO_START_VARS.initialize();
            }
        } else {
            currStage.function();
        }
    }

    public boolean done() {
        return stages.isEmpty();
    }
}


