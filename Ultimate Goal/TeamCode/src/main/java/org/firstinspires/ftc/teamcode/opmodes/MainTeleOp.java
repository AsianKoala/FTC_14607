package org.firstinspires.ftc.teamcode.opmodes;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import static org.firstinspires.ftc.teamcode.movement.PPController.*;
import org.firstinspires.ftc.teamcode.hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.movement.CurvePoint;
import org.firstinspires.ftc.teamcode.movement.Odometry;
import org.firstinspires.ftc.teamcode.movement.PPController;
import org.firstinspires.ftc.teamcode.util.Point;
import org.firstinspires.ftc.teamcode.util.Pose;

import java.util.ArrayList;

@TeleOp(name="main teleop")
public class MainTeleOp extends Robot {

    private Point anglePoint;
    private boolean headingControlled = false;
    private boolean turnToGoal = false;
    private boolean goToShootingPoint = false;

    // shooting vars
    private Pose stageStartPose;
    private boolean stageFinished = true;
    private void initVarsForMove() {
        stageFinished = false;
        stageStartPose = Odometry.currentPosition;
    }


    @Override
    public void init() {
        super.init();
        odometry.setStart(new Pose(0, -24, Math.toRadians(180)));
        anglePoint = new Point(0,0);
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
        controlMovement();
        controlAnglePoint();
        controlShootingMovement();
        telemetryVars();
    }


    public void controlMovement() {
        double driveScale = 0.5 + (gamepad1.right_bumper ? 0.5 : 0) - (gamepad1.left_bumper ? 0.35 : 0);
        DriveTrain.movementY = -gamepad1.left_stick_y * driveScale;
        DriveTrain.movementX = gamepad1.left_stick_x * driveScale;
        DriveTrain.movementTurn = -gamepad1.right_stick_x * driveScale;
    }

    @SuppressLint("DefaultLocale")
    public void controlAnglePoint() {
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
            headingControlled = !headingControlled;
        }

        if(headingControlled) {
            PPController.movementResult result = PPController.pointPointTurn(anglePoint, 0.8, Math.toRadians(20));
            telemetry.addLine(String.format("movementR: %.2f", Math.toDegrees(result.turnDelta_rad)));
        }


        if(gamepad1.left_trigger > 0.7) {
            turnToGoal = true;
        }

        if(turnToGoal) {
            PPController.movementResult result = PPController.pointAngle(Math.toRadians(90), 0.8, Math.toRadians(20));
            telemetry.addLine(String.format("movementR: %.2f", Math.toDegrees(result.turnDelta_rad)));

            if(result.turnDelta_rad < Math.toRadians(2)) {
                turnToGoal = false;
            }
        }
    }

    public void controlShootingMovement() {
        // big one
        if(gamepad1.right_trigger > 0.7) {
            goToShootingPoint = true;
        }

        if(goToShootingPoint) {
            if(stageFinished) {
                initVarsForMove();
            }

            ArrayList<CurvePoint> allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(stageStartPose.x, stageStartPose.y, 0, 0, 0, 0, 0, 0));
            allPoints.add(new CurvePoint(0,  stageStartPose.y / 2.0, 0.6, 0.6, 15, 20, Math.toRadians(45), 0.6));
            allPoints.add(new CurvePoint(0, -4, 0.5, 0.5, 10, 15, Math.toRadians(45), 0.85));
            boolean done = betterFollowCurve(allPoints, Math.toRadians(90), null, false);

            if(done) {
                DriveTrain.stopMovement();
                boolean turnDone = pointAngle(Math.toRadians(90), 0.6, Math.toRadians(30)).turnDelta_rad < Math.toRadians(2);

                if(turnDone) {
                    DriveTrain.stopMovement();
                    goToShootingPoint = false;
                    stageFinished = true;
                }
            }
        }
    }


    public void telemetryVars() {
        telemetry.addLine("anglePoint: " + anglePoint.toString());
        telemetry.addLine("headingControlled: " + headingControlled);
        telemetry.addLine("turnToGoal: " + turnToGoal);
        telemetry.addLine("goToShootingPoint: " + goToShootingPoint);
    }


}
