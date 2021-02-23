package org.firstinspires.ftc.teamcode.movement;

import org.firstinspires.ftc.teamcode.control.Modifier;

public class Sequence {
    public PurePursuitPath path;
    public boolean isMovementPath;

    public Sequence(PurePursuitPath path) {
        this.path = path;

        isMovementPath = false;
        for (BaseCurvePoint c : path.allPoints) {
            if (!(c.function instanceof Modifier.hardwareFunction)) {
                isMovementPath = true;
                break;
            }
        }
    }
}
