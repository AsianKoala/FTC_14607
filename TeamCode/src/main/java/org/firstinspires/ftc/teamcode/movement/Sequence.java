package org.firstinspires.ftc.teamcode.movement;

import org.firstinspires.ftc.teamcode.control.Functions;

public class Sequence {
    public PurePursuitPath path;
    public boolean isMovementPath;

    public Sequence(PurePursuitPath path) {
        this.path = path;

        isMovementPath = false;
        for (BaseCurvePoint c : path.allPoints) {
            if (!(c.function instanceof Functions.hardwareFunction)) {
                isMovementPath = true;
                break;
            }
        }
    }
}