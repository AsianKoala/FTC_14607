package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.movement.CurvePoint;
import static org.firstinspires.ftc.teamcode.movement.PPController.*;
import org.firstinspires.ftc.teamcode.util.*;
import static org.firstinspires.ftc.teamcode.hardware.DriveTrain.*;
import static org.firstinspires.ftc.teamcode.movement.Odometry.*;

import java.util.ArrayList;

@Autonomous(name="main auto")
public class MainAuto extends Auto {
    public enum programStates {
        moveToPosition,
        initialShots,
        collectRings,
        secondShots,
        placeMarker,
        goToSecondMarker,
        dropOffSecondMarker,
        park
    }


    @Override
    public void init() {
        super.init();
        odometry.setStart(new Pose(0,-63, Math.toRadians(180)));
        setStage(programStates.moveToPosition.ordinal());
    }

    @Override
    public void init_loop() {
        super.init_loop();
    }

    @Override
    public void start() {
        super.start();
        setStage(programStates.moveToPosition.ordinal());
    }

    @Override
    public void loop() {
        super.loop();
        telemetry.addLine("Curr stage: " + programStates.values()[progState]);
    }

    @Override
    public void autoStateMachine() {
        if(progState == programStates.moveToPosition.ordinal()) {
            if(stageFinished) {
                initProgVars();
            }

            ArrayList<CurvePoint> allPoints = new ArrayList<>();
            allPoints.add(initialCurvePoint());

            if (ringAmount != RingDetectorPipeline.RingAmount.NONE) {
                allPoints.add(new CurvePoint(30, -30, 0.6, 0.6, 15, 20, Math.toRadians(60), 0.7));
                allPoints.add(new CurvePoint(15, -10, 0.6, 0.6, 15, 20, Math.toRadians(60), 0.7));
            }

            allPoints.add(new CurvePoint(-3, -6, 0.6, 0.6, 20, 25, Math.toRadians(60), 0.7));
            boolean isDone = betterFollowCurve(allPoints, Math.toRadians(90), new Point(3.5, 48), true);

            if(isDone) {
                stopMovement();
                odometry.setGlobalPosition(new Point(0, 0));
                nextStage();
            }
        }


        if(progState == programStates.initialShots.ordinal()) {
            if(stageFinished) {
                initProgVars();
            }

            // turn on intake and shoot

            if(timeCheck(3000)) {
                if(ringAmount == RingDetectorPipeline.RingAmount.NONE) {
                    setStage(programStates.placeMarker.ordinal());
                } else {
                    nextStage();
                }
            }
        }


        if(progState == programStates.collectRings.ordinal()) {
            if (stageFinished) {
                initProgVars();
            }

            ArrayList<CurvePoint> allPoints = new ArrayList<>();
            allPoints.add(initialCurvePoint());
            allPoints.add(new CurvePoint(0, -24, 0.2, 0.3, 10, 15, Math.toRadians(60), 0.8));
            boolean done = betterFollowCurve(allPoints, Math.toRadians(90), new Point(2, 48), true);

            if (done) {
                stopMovement();
                nextStage();
            }
        }


        if(progState == programStates.secondShots.ordinal()) {
            if(stageFinished) {
                initProgVars();
            }

            ArrayList<CurvePoint> allPoints = new ArrayList<>();
            allPoints.add(initialCurvePoint());
            allPoints.add(new CurvePoint(0, 0, 0.6, 0.6, 10, 15, Math.toRadians(60), 0.6));
            boolean done = betterFollowCurve(allPoints, Math.toRadians(90), new Point(2, 48), true);

            if(done) {
                // start shooting
                stopMovement();
            }

            if(timeCheck(3000)) {
                nextStage();
            }
        }


        if(progState == programStates.placeMarker.ordinal()) {
            if(stageFinished) {
                initProgVars();
            }

            ArrayList<CurvePoint> allPoints = new ArrayList<>();
            allPoints.add(initialCurvePoint());

            switch(ringAmount) {
                case NONE:
                    allPoints.add(new CurvePoint(-12,12, 0.5, 0.5, 10, 15, Math.toRadians(45), 0.8));
                    break;
                case ONE:
                    allPoints.add(new CurvePoint(4, 20, 0.5, 0.5, 20, 25, Math.toRadians(60), 0.6));
                    break;
                case FOUR:
                    allPoints.add(new CurvePoint(-13, 36, 0.5, 0.5, 20, 25, Math.toRadians(60), 0.6));
                    break;
            }
            boolean done = betterFollowCurve(allPoints, Math.toRadians(90), null, false);

            if(done) {
                stopMovement();
                if(timeCheck(2000)) {
                    nextStage();
                }
            }
        }


        if(progState == programStates.goToSecondMarker.ordinal()) {
            if(stageFinished) {
                initProgVars();
            }
            ArrayList<CurvePoint> allPoints = new ArrayList<>();
            allPoints.add(initialCurvePoint());

            double markerX = -36;
            double scale = Range.clip((currentPosition.x - markerX)/24.0, 0.1, 1.0);
            allPoints.add(new CurvePoint(-6, 2, 0.6 * scale, 0.6, 20, 25, Math.toRadians(60), 0.6));
            allPoints.add(new CurvePoint(-6, -42, 0.6 * scale, 0.6, 20, 25, Math.toRadians(60), 0.6));
            boolean done = betterFollowCurve(allPoints, Math.toRadians(90), null, false);

            if(done) {
                // pick up with servo or somethign idfk
                if (timeCheck(4500)) {
                    stopMovement();
                    nextStage();
                }
            }
        }

        if(progState == programStates.dropOffSecondMarker.ordinal()) {
            if(stageFinished) {
                initProgVars();
            }

            ArrayList<CurvePoint> allPoints = new ArrayList<>();
            allPoints.add(initialCurvePoint());

            switch(ringAmount) {
                case NONE:
                    allPoints.add(new CurvePoint(-6,12, 0.6, 0.6, 20, 25, Math.toRadians(45), 0.6));
                    break;
                case ONE:
                    allPoints.add(new CurvePoint(4, 20, 0.6, 0.6, 20, 25, Math.toRadians(60), 0.6));
                    break;
                case FOUR:
                    allPoints.add(new CurvePoint(-10, 34, 0.6, 0.6, 20, 25, Math.toRadians(60), 0.6));
                    break;
            }
            boolean done = betterFollowCurve(allPoints, Math.toRadians(90), null, false);

            if(done) {
                stopMovement();
                if(timeCheck(2000)) {
                    nextStage();
                }
            }
        }


        if(progState == programStates.park.ordinal()) {
            if(stageFinished) {
                initProgVars();
            }
            ArrayList<CurvePoint> allPoints = new ArrayList<>();
            allPoints.add(initialCurvePoint());
            allPoints.add(new CurvePoint(0, -3, 0.5, 0.5, 20, 25, Math.toRadians(30), 0.6));
            boolean done = betterFollowCurve(allPoints, Math.toRadians(90), null, false);

            if(ringAmount == RingDetectorPipeline.RingAmount.NONE) {
                done = true;
            }

            if(done) {
                stopMovement();
                requestOpModeStop();
            }
        }


    }
}
