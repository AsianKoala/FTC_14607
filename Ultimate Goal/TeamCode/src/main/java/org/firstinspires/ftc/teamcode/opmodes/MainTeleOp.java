package org.firstinspires.ftc.teamcode.opmodes;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import static org.firstinspires.ftc.teamcode.movement.PPController.*;
import org.firstinspires.ftc.teamcode.hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.movement.Odometry;
import org.firstinspires.ftc.teamcode.movement.PPController;
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.Point;
import org.firstinspires.ftc.teamcode.util.Pose;

import java.util.ArrayList;

@TeleOp(name="main teleop")
public class MainTeleOp extends Robot {

    public final Point anglePoint = new Point(0, 72);
    public boolean anglePointControlled = false;
    public boolean turnToGoal = false;
    public boolean goToShootingPoint = false;

    private boolean turnReached = false;
    private boolean shootingZoneReached = false;



    @Override
    public void init() {
        super.init();
        odometry.setStart(new Pose(0, 0, Math.toRadians(90)));
    }

    @Override
    public void init_loop() {
        super.init_loop();
    }

    @Override
    public void start() {
        super.start();
    }


    @SuppressLint("DefaultLocale")
    @Override
    public void loop() {
        super.loop();
        controlJoystickMovement();
        controlAnglePointMovement();
        controlShootingMovement();
        telemetryVars();
    }


    public void controlJoystickMovement() {
        double driveScale = 0.5 + (gamepad1.right_bumper ? 0.5 : 0) - (gamepad1.left_bumper ? 0.2 : 0);
        DriveTrain.movementY = -gamepad1.left_stick_y * driveScale;
        DriveTrain.movementX = gamepad1.left_stick_x * driveScale;
        DriveTrain.movementTurn = -gamepad1.right_stick_x * driveScale;
    }

    @SuppressLint("DefaultLocale")
    public void controlAnglePointMovement() {
        if(gamepad1.dpad_right) {
            anglePoint.x += 6;
        }
        if(gamepad1.dpad_left) {
            anglePoint.x -= 6;
        }
        if(gamepad1.dpad_up) {
            anglePoint.y += 6;
        }
        if(gamepad1.dpad_down) {
            anglePoint.y -= 6;
        }

        if(gamepad1.y) {
            anglePointControlled = !anglePointControlled;
        }

        if(anglePointControlled) {
            PPController.movementResult result = PPController.pointPointTurn(anglePoint, 0.8, Math.toRadians(40));
            telemetry.addLine(String.format("movementR: %.2f", Math.toDegrees(result.turnDelta_rad)));
        }


        if(gamepad1.left_trigger > 0.7) {
            turnToGoal = true;
        }

        if(turnToGoal) {
            PPController.movementResult result = pointAngle(Math.toRadians(90), 0.8, Math.toRadians(45));
            telemetry.addLine(String.format("movementR: %.2f", Math.toDegrees(result.turnDelta_rad)));

            if(isTurnedTowardsGoal()) {
                turnToGoal = false;
            }
        }
    }


    /**
     * Possible problems:
     * -    DriveTrain.stopMovement
     * -    Most likely problem: turnDeltaRad or movementResult are fucked so instead just do smth else
     */

    public void controlShootingMovement() {
        if (gamepad1.right_trigger > 0.7) {
            goToShootingPoint = true;
        }

        if(goToShootingPoint) {
            if(!shootingZoneReached) {
                boolean done = goToPosition(0, 0, 0.6, Math.toRadians(90), 0.6, Math.toRadians(45), 0.6, 2, true).withinBounds;
                if(done) {
                    DriveTrain.stopMovement();
                    shootingZoneReached = true;
                }
            } else if(!turnReached) {
                pointAngle(Math.toRadians(90), 0.8, Math.toRadians(45));
                if(isTurnedTowardsGoal()) {
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
        return Math.abs(MathUtil.angleWrap(Math.toRadians(90) - Odometry.currentPosition.heading)) < Math.toRadians(2);
    }

//

    public void telemetryVars() {
        telemetry.addLine("anglePoint: " + anglePoint.toString());
        telemetry.addLine("headingControlled: " + anglePointControlled);
        telemetry.addLine("turnToGoal: " + turnToGoal);
        telemetry.addLine("goToShootingPoint: " + goToShootingPoint);
        telemetry.addLine("90diff: " + Math.toDegrees(MathUtil.angleWrap(Math.toRadians(90) - Odometry.currentPosition.heading)));
        telemetry.addLine("shootingZoneReached: " + shootingZoneReached);
        telemetry.addLine("turnReached: " + turnReached);
    }


}
