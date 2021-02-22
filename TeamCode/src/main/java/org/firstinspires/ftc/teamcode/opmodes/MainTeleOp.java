package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.control.OurOpMode;
import org.firstinspires.ftc.teamcode.hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.movement.Odometry;
import org.firstinspires.ftc.teamcode.util.Util;

import static org.firstinspires.ftc.teamcode.movement.MovementController.goToPosition;
import static org.firstinspires.ftc.teamcode.movement.MovementController.pointAngle;

@TeleOp(name = "teleop")
public class MainTeleOp extends OurOpMode {

    private boolean goToShootingPoint = false;
    private boolean turnReached = false;
    private boolean shootingZoneReached = false;

    private final boolean intakeOn = false;

    private final double actuatorStartTime = 0;
    private final boolean shootingStateMachine = false;
    private final double shootingStateMachineStartTime = 0;
    private boolean shooterOn = false;

    @Override
    public void init() {
        super.init();
    }

    @Override
    public void loop() {
        super.loop();
        controlJoystickMovement();
        controlShooter();
        controlActuator();
        controlWobbleGoal();
        controlShootingMovement();
        telemetryVars();
    }

    public void controlJoystickMovement() {
        double driveScale = 0.5 - (gamepad1.left_bumper ? 0.2 : 0);
        DriveTrain.movementY = -gamepad1.left_stick_y * driveScale;
        DriveTrain.movementX = gamepad1.left_stick_x * driveScale;
        DriveTrain.movementTurn = -gamepad1.right_stick_x * driveScale;
    }

    public void controlShooter() {
        if (gamepad1.left_trigger > 0.7) {
            shooterOn = !shooterOn;
        }
        if (shooterOn) {
            shooter.turnOn();
        }
    }

    public void controlActuator() {
        if (gamepad1.a) {
            actuator.push();
        }
        if (gamepad1.b) {
            actuator.reset();
        }
    }

    public void controlWobbleGoal() {
        if (gamepad1.dpad_up) {
            wobbleGoal.in();
        }
        if (gamepad1.dpad_down) {
            wobbleGoal.out();
        }
        if (gamepad1.dpad_left) {
            wobbleGoal.grab();
        }
        if (gamepad1.dpad_right) {
            wobbleGoal.release();
        }
    }


    public void controlShootingMovement() {
        if (gamepad1.right_trigger > 0.7) {
            goToShootingPoint = true;
        }

        if (goToShootingPoint) {
            if (!shootingZoneReached) {
                boolean done = goToPosition(0, 0, 0.6, Math.toRadians(90), 0.6, Math.toRadians(45), 0.6, 2, true).done;
                if (done) {
                    DriveTrain.stopMovement();
                    shootingZoneReached = true;
                }
            } else if (!turnReached) {
                pointAngle(Math.toRadians(90), 0.8, Math.toRadians(45));
                if (isTurnedTowardsGoal()) {
                    DriveTrain.stopMovement();
                    turnReached = true;
                }
            } else {
                DriveTrain.stopMovement();
                shootingZoneReached = false;
                turnReached = false;
                goToShootingPoint = false;
            }
        }
    }


    private boolean isTurnedTowardsGoal() {
        return Math.abs(Util.angleWrap(Math.toRadians(90) - Odometry.currentPosition.heading)) < Math.toRadians(2);
    }


    public void telemetryVars() {
        telemetry.addLine("goToShootingPoint: " + goToShootingPoint);
        telemetry.addLine("90diff: " + Math.toDegrees(Util.angleWrap(Math.toRadians(90) - Odometry.currentPosition.heading)));
        telemetry.addLine("shootingZoneReached: " + shootingZoneReached);
        telemetry.addLine("turnReached: " + turnReached);
    }
}
