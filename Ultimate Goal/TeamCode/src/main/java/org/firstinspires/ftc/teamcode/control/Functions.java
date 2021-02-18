package org.firstinspires.ftc.teamcode.control;


public class Functions {
    public interface function {
        Results.baseResult result();
    }

    public interface hardwareFunction extends function {
        Results.simpleResult runHardware();
    }

    public interface movementFunction extends function {
        Results.movementResult runMovement();
    }

    public interface pointToPointFunction extends function {
        Results.movementResult runTurn();
    }

    public interface headingControlledFunction extends function {
        Results.movementResult runTurn();
    }

    public interface complexFunction extends function {
        Results.complexResult runComplexFunctions();
    }
}


