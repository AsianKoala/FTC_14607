package org.firstinspires.ftc.teamcode.PepeLaugh.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.PepeLaugh.Debugging.TimeProfiler;
import org.firstinspires.ftc.teamcode.PepeLaugh.Globals.Globals;
import org.firstinspires.ftc.teamcode.PepeLaugh.HelperClasses.Auto;
import android.os.SystemClock;
import org.firstinspires.ftc.teamcode.PepeLaugh.RobotUtilities.MyPosition;
import static org.firstinspires.ftc.teamcode.PepeLaugh.RobotUtilities.MovementEssentials.gunToPosition;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.PepeLaugh.HelperClasses.CurvePoint;
import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.PepeLaugh.RobotUtilities.MovementEssentials.followCurve;

@Autonomous(name = "testOpMode", group = "test")
public class testOpMode extends Auto {

    private double SPEED_SCALE = 2.1;

    public enum progStates{
        unlatching,
        firstMove,
        secondMove,

        endDoNothing
    }

    /**
     * When driver presses init
     */
    @Override
    public void init() {
        Globals.autoMode = Globals.AutoModes.auto2;

        super.init();

    }

    @Override
    public void init_loop() {
        super.init_loop();
    }

    @Override
    public void start() {
        startDebugging();

        nextStage(progStates.unlatching.ordinal());

    }

    private static ArrayList<TimeProfiler> allTimeProfilers = new ArrayList<>();

    static {
        for(int i=0; i<10; i++) {
            allTimeProfilers.add(new TimeProfiler(500));
        }
    }

    @Override
    public void loop() {

        allTimeProfilers.get(3).markEnd();
        allTimeProfilers.get(3).markStart();
        telemetry.addLine("UPS: " + 1000.0/allTimeProfilers.get(3).getAverageTimePerUpdateMillis());





        allTimeProfilers.get(0).markStart();
        super.loop();
        allTimeProfilers.get(0).markEnd();


        allTimeProfilers.get(1).markStart();


        allTimeProfilers.get(1).markEnd();




        allTimeProfilers.get(2).markStart();

        allTimeProfilers.get(2).markEnd();

        telemetry.addLine("average time profilier 0: " + allTimeProfilers.get(0).getAverageTimePerUpdateMillis());
        telemetry.addLine("average time profilier 1: " + allTimeProfilers.get(1).getAverageTimePerUpdateMillis());
        telemetry.addLine("average time profilier 2: " + allTimeProfilers.get(2).getAverageTimePerUpdateMillis());
        telemetry.addLine("average time profilier 3: " + allTimeProfilers.get(3).getAverageTimePerUpdateMillis());
    }

    // this is where the juice is

    @Override
    public void MainStateMachine() {
        super.MainStateMachine();

        ArrayList<CurvePoint> allPoints = new ArrayList<>();

        //add our initial location, parameters don't matter since this only defines the shape of the first line

        allPoints.add(new CurvePoint(blockStartingX,blockStartingY,
                0,0,0,0,0));

        allPoints.add(new CurvePoint(140,115,
                0.5*SPEED_SCALE,0.5*SPEED_SCALE,50,50,
                Math.toRadians(60),0.6));

        allPoints.add(new CurvePoint(155,65,
                0.4*SPEED_SCALE, 0.5*SPEED_SCALE,50,65,
                Math.toRadians(60),0.6));



        // now we want to slow down as we get to the target point so we do this
        double target = 290;
        double myX = getXPos();


        double scaleDownLastMove = (1.0* Range.clip((target = myX )
                /100.0,0.05,1.0));


        allPoints.add(new CurvePoint(190, 30.5,
                0.5*SPEED_SCALE * scaleDownLastMove,0.5*SPEED_SCALE,50,65,
                Math.toRadians(60),0.6));



        allPoints.add(new CurvePoint(285, 30.5,
                0.4 * scaleDownLastMove*SPEED_SCALE,0.6*SPEED_SCALE,50,65,
                Math.toRadians(60),0.6));

        followCurve(allPoints,Math.toRadians(90),false);

        nextStage();

        gunToPosition(FIELD_LENGTH-115,115,
                Math.toRadians(-45),0.3*SPEED_SCALE,0.3*SPEED_SCALE,
                Math.toRadians(30),0.5,false);

        nextStage();

        stopMovement();
    }
}
