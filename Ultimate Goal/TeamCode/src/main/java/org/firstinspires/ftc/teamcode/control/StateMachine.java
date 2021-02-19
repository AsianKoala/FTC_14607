package org.firstinspires.ftc.teamcode.control;

import java.util.LinkedList;
import static org.firstinspires.ftc.teamcode.control.Stages.*;


public class StateMachine {
    public LinkedList<Stages.basicStage> stages;
    public Stages.basicStage currStage;
    public int completedStages;

    public StateMachine() {
        stages = new LinkedList<>();
        completedStages = -1;
    }

    public void addStage(Stages.basicStage newStage) {
        stages.add(newStage);
    }

    public boolean running() {
        return !stages.isEmpty();
    }

    public void skipStages(int newStageIndex) {
        // convert string to int
        for(int i=0; i<newStageIndex-completedStages; i++)
            stages.removeFirst();
        currStage = stages.getFirst();
    }

    public void loop() {
        if(completedStages == -1) {
            currStage = stages.getFirst();
            System.out.println(currStage.name());
            currStage.AUTO_START_VARS.initialize();
            currStage.startFunction();
            completedStages++;
        }

        if(currStage.mainFunction().done) {
            currStage.endFunction();
            stages.removeFirst();
            completedStages++;
            if(running()) {
                currStage = stages.getFirst();
                currStage.AUTO_START_VARS.initialize();
                currStage.startFunction();
            }
        }
    }
}