package org.firstinspires.ftc.teamcode.control;

import java.util.LinkedList;
import static org.firstinspires.ftc.teamcode.control.Stages.*;


public class StateMachine {
    public LinkedList<basicStage> stages;
    public basicStage currStage;
    public int completedStages;

    public StateMachine() {
        stages = new LinkedList<>();
    }

    public void addStage(basicStage newStage) {
        stages.add(newStage);
        currStage = stages.getFirst();
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
        if(currStage.finishState()) {
            currStage.endFunction();
            stages.removeFirst();
            completedStages++;
            if(running()) {
                currStage = stages.getFirst();
                currStage.AUTO_START_VARS.initialize();
                currStage.startFunction();
            }
        } else {
            currStage.mainFunction();
        }
    }
}


class testStateMachine extends auto {
    void init() {
        super.init();
    }

    void loop() {
        super.loop();
    }


    @Override
    StateMachine machine() {
        StateMachine returnMachine = new StateMachine();
        returnMachine.addStage(new basicStage() {

            @Override
            public boolean finishState() {
                return false;
            }

            @Override
            public void startFunction() {

            }

            @Override
            public void mainFunction() {

            }

            @Override
            public void endFunction() {

            }

            @Override
            public String name() {
                return null;
            }
        });
        return returnMachine;
    }
}


abstract class auto {
    abstract StateMachine machine();

    void init() {

    }

    void loop() {
        if(machine().running()) {
            machine().loop();
            System.out.println("stage " + machine().completedStages + ": " + machine().currStage.name());
        } else {
            System.exit(0);
        }
    }


}
















