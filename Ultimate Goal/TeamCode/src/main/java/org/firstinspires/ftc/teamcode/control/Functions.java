package org.firstinspires.ftc.teamcode.control;


import java.util.ArrayList;

public class Functions {
    public static abstract class function {
        boolean startCondition;
        
        abstract Results.baseResult result();
    }

    public abstract static class hardwareFunction extends function {
        abstract Results.simpleResult runHardware();

        @Override
        Results.baseResult result() {
            return runHardware();
        }
    }

    public abstract static class movementFunction extends function {
        abstract Results.movementResult runMovement();

        @Override
        Results.baseResult result() {
            return runMovement();
        }
    }

    public abstract static class pointToPointFunction extends function {
        abstract Results.movementResult runTurn();

        @Override
        Results.baseResult result() {
            return runTurn();
        }
    }

    public abstract static class headingControlledFunction extends function {
        abstract Results.movementResult runTurn();

        @Override
        Results.baseResult result() {
            return runTurn();
        }
    }

    public abstract static class complexFunction extends function {
        abstract ArrayList<function> functions();

        Results.baseResult runComplexFunctions() {
            boolean isDone = true;
            int completedFunctions = 0;
            for(function f : functions()) {
                if(f.startCondition) {
                    if(!f.result().done)
                        isDone = false;
                }
            }
            Results.baseResult returnResult = new Results.baseResult();
            returnResult.done = isDone;
            return returnResult;
        }

        @Override
        Results.baseResult result() {
            return runComplexFunctions();
        }
    }
}


