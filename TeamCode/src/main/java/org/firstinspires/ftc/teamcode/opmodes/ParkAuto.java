package org.firstinspires.ftc.teamcode.main.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.main.control.BaseAuto;
import org.firstinspires.ftc.teamcode.main.control.Results;
import org.firstinspires.ftc.teamcode.main.hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.main.movement.MovementController;
import org.firstinspires.ftc.teamcode.main.util.Pose;


@Autonomous(name = "park auto")
public class ParkAuto extends BaseAuto {
    @Override
    public void init() {
        super.init();
        odometry.setStart(new Pose(0, -64, Math.toRadians(90)));
        setState(stateMachineStates.park.ordinal());
    }

    @Override
    public void init_loop() {
        super.init_loop();
    }

    @Override
    public void start() {
        super.start();
        setState(stateMachineStates.park.ordinal());
    }

    @Override
    public void loop() {
        super.loop();
        telemetry.addLine("Curr state: " + stateMachineStates.values()[currState]);
    }

    @Override
    public void autoStateMachine() {
        if (currState == stateMachineStates.park.ordinal()) {
            if (stateFinished) {
                initStateVars();
            }

            Results.movementResult result = MovementController.goToPosition(0, 12, 0.6, 0.6, Math.toRadians(90),
                    Math.toRadians(45), 0.6, 2.5, true);

            if (result.done) {
                DriveTrain.stopMovement();
                requestOpModeStop();
            }
        }
    }

    public enum stateMachineStates {
        park,
        stopOpMode
    }
}
