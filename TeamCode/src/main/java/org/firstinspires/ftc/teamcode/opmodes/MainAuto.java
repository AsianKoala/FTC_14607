package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.control.Auto;
import org.firstinspires.ftc.teamcode.control.Results;
import org.firstinspires.ftc.teamcode.hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.movement.BaseCurvePoint;
import org.firstinspires.ftc.teamcode.movement.MovementController;
import org.firstinspires.ftc.teamcode.util.Pose;

import java.util.ArrayList;

@Autonomous(name = "auto")
public class MainAuto extends Auto {

    private double actuatorStartTime = 0;
    private double shootingStateMachineStartTime = 0;

    @Override
    public void init() {
        super.init();
        odometry.setStart(new Pose(0, -64, Math.toRadians(180)));
    }

    @Override
    public void init_loop() {
        super.init_loop();
    }

    @Override
    public void start() {
        super.start();
        setState(programStates.goToShootingPosition.ordinal());
    }

    @Override
    public void loop() {
        super.loop();
        telemetry.addLine("Curr state: " + programStates.values()[currState]);
    }

    @Override
    public void autoStateMachine() {
        if (currState == programStates.goToShootingPosition.ordinal()) {
            if (stateFinished) {
                initStateVars();
            }

            Results.movementResult result = MovementController.goToPosition(0, 0, 0.6, 0.6, Math.toRadians(90),
                    Math.toRadians(45), 0.6, 2.5, true);
            if (result.done) {
                DriveTrain.stopMovement();
                nextState();
            }
        }

        if (currState == programStates.shoot1.ordinal()) {
            if (stateFinished) {
                initStateVars();
                initShooter();
            }
            shoot();
            if (timeCheck(2500)) {
                nextState();
            }
        }
        if (currState == programStates.shoot2.ordinal()) {
            if (stateFinished) {
                initStateVars();
                initShooter();
            }
            shoot();
            if (timeCheck(2500)) {
                nextState();
            }
        }
        if (currState == programStates.shoot3.ordinal()) {
            if (stateFinished) {
                initStateVars();
                initShooter();
            }
            shoot();
            if (timeCheck(2500)) {
                nextState();
            }
        }

        if (currState == programStates.goToWobbleDrop.ordinal()) {
            if (stateFinished) {
                initStateVars();
            }

            ArrayList<BaseCurvePoint> allPoints = new ArrayList<>();
            allPoints.add(initialCurvePoint());

            switch (ringAmount) {
                case NONE:
                    allPoints.add(new BaseCurvePoint(-12, 12, 0.5, 0.5, 10, 15, Math.toRadians(45), 0.8));
                    break;
                case ONE:
                    allPoints.add(new BaseCurvePoint(4, 20, 0.5, 0.5, 20, 25, Math.toRadians(60), 0.6));
                    break;
                case FOUR:
                    allPoints.add(new BaseCurvePoint(-13, 36, 0.5, 0.5, 20, 25, Math.toRadians(60), 0.6));
                    break;
            }
            boolean done = betterFollowCurve(allPoints, Math.toRadians(90), null, false, 0);

            if (done) {
                DriveTrain.stopMovement();
                nextState();
            }
        }

        if (currState == programStates.dropWobble.ordinal()) {
            if (stateFinished) {
                initStateVars();
            }

            wobbleGoal.out();
            wobbleGoal.release();

            if (timeCheck(2000)) {
                nextState();
                wobbleGoal.in();
            }
        }

        if (currState == programStates.goToSecondWobbleGoal.ordinal()) {
            if (stateFinished) {
                initStateVars();
            }

            ArrayList<BaseCurvePoint> allPoints = new ArrayList<>();
            allPoints.add(initialCurvePoint());
            allPoints.add(new BaseCurvePoint(0, 2, 0.6, 0.6, 20, 25, Math.toRadians(60), 0.6));
            allPoints.add(new BaseCurvePoint(0, -42, 0.6, 0.6, 20, 25, Math.toRadians(60), 0.6));
            boolean done = MovementController.betterFollowCurve(allPoints, Math.toRadians(90), null, false, 0);
            if (done) {
                DriveTrain.stopMovement();
                nextState();
            }
        }

        if (currState == programStates.grabWobbleGoal.ordinal()) {
            if (stateFinished) {
                initStateVars();
            }
            wobbleGoal.out();
            if (timeCheck(1000)) {
                nextState();
            }
        }

        if (currState == programStates.pullWobble.ordinal()) {
            if (stateFinished) {
                initStateVars();
            }
            wobbleGoal.grab();
            wobbleGoal.in();
            if (timeCheck(1000)) {
                nextState();
            }
        }

        if (currState == programStates.goToSecondWobbleGoal.ordinal()) {
            if (stateFinished) {
                initStateVars();
            }

            ArrayList<BaseCurvePoint> allPoints = new ArrayList<>();
            allPoints.add(initialCurvePoint());

            switch (ringAmount) {
                case NONE:
                    allPoints.add(new BaseCurvePoint(-6, 12, 0.6, 0.6, 20, 25, Math.toRadians(45), 0.6));
                    break;
                case ONE:
                    allPoints.add(new BaseCurvePoint(4, 20, 0.6, 0.6, 20, 25, Math.toRadians(60), 0.6));
                    break;
                case FOUR:
                    allPoints.add(new BaseCurvePoint(-10, 34, 0.6, 0.6, 20, 25, Math.toRadians(60), 0.6));
                    break;
            }
            boolean done = betterFollowCurve(allPoints, Math.toRadians(90), null, false, 0);
            if (done) {
                DriveTrain.stopMovement();
                nextState();
            }
        }

        if (currState == programStates.dropSecondWobble.ordinal()) {
            if (stateFinished) {
                initStateVars();
            }

            wobbleGoal.release();
            wobbleGoal.out();
            if (timeCheck(1000)) {
                nextState();
            }
        }

        if (currState == programStates.park.ordinal()) {
            if (stateFinished) {
                initStateVars();
            }
            Results.movementResult result = MovementController.goToPosition(0, 0, 0.6, 0.6, Math.toRadians(90), Math.toRadians(60),
                    0.6, 3, true);
            if (result.done) {
                DriveTrain.stopMovement();
                requestOpModeStop();
            }
        }
    }

    private void shoot() {
        if (System.currentTimeMillis() - shootingStateMachineStartTime > 1000) { // rev time
            actuatorStartTime = System.currentTimeMillis();
            actuator.push();
        }

        if (System.currentTimeMillis() - actuatorStartTime > 1500) { // push time
            actuator.reset();
            shooter.turnOff();
        }
    }

    private void initShooter() {
        shootingStateMachineStartTime = System.currentTimeMillis();
        shooter.turnOn();
    }

    private enum programStates {
        goToShootingPosition,
        shoot1, shoot2, shoot3,
        goToWobbleDrop,
        dropWobble,
        goToSecondWobbleGoal,
        grabWobbleGoal,
        pullWobble,
        goToSecondDrop,
        dropSecondWobble,
        park
    }
}
