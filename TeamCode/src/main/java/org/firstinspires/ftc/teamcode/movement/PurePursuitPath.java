package org.firstinspires.ftc.teamcode.main.movement;

import java.util.ArrayList;

public class PurePursuitPath {
    public ArrayList<CurvePoint> allPoints;
    public boolean skip;

    public PurePursuitPath(ArrayList<CurvePoint> allPoints, boolean skip) {
        this.allPoints = allPoints;
        this.skip = skip;
    }

}
