package org.firstinspires.ftc.teamcode.lastMinutescuffedPP.teemode;

import org.firstinspires.ftc.teamcode.lastMinutescuffedPP.HelperClasses.CurvePoint;
import static org.firstinspires.ftc.teamcode.lastMinutescuffedPP.Movement.RobotMovement.followCurve;
import java.util.ArrayList;

public class DebuggingTest extends DebuggingOpMode {
    @Override
    public void init() {

    }

    @Override
    public void loop() {
        ArrayList<CurvePoint> allPoints = new ArrayList<>();
        allPoints.add(new CurvePoint(180,180,1.0,1,20,Math.toRadians(50),1.0));
        allPoints.add(new CurvePoint(220,180,1,1,20, Math.toRadians(50),1.0));
        allPoints.add(new CurvePoint(280,100,1,1,20, Math.toRadians(50),1.0));
        allPoints.add(new CurvePoint(280,0,1,1,20, Math.toRadians(50),1.0));
        followCurve(allPoints, Math.PI/2);
    }
}
