package org.firstinspires.ftc.teamcode.opmodes;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import static org.firstinspires.ftc.teamcode.movement.PPController.*;
import org.firstinspires.ftc.teamcode.hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.movement.CurvePoint;
import org.firstinspires.ftc.teamcode.movement.Odometry;
import org.firstinspires.ftc.teamcode.movement.PPController;
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.Point;
import org.firstinspires.ftc.teamcode.util.Pose;

import java.util.ArrayList;

@TeleOp(name="main teleop")
public class MainTeleOp extends Robot {

//    public final Point anglePoint = new Point(0, 72);
//    public boolean anglePointControlled = false;
    public boolean turnToGoal = false;
    public boolean goToShootingPoint = false;

    // shooting vars

//    private enum shootingStages {
//        MOVING,
//        TURNING
//    }
//
//    private shootingStages currStage;
//    private void nextStage() {
//        DriveTrain.stopMovement();
//        if(currStage == shootingStages.MOVING) {
//            currStage = shootingStages.TURNING;
//        } else {
//            goToShootingPoint = false;
//            currStage = shootingStages.MOVING;
//        }
//    }



    @Override
    public void init() {
        super.init();
        odometry.setStart(new Pose(24, -36, Math.toRadians(180)));
    }

    @Override
    public void init_loop() {
        super.init_loop();
    }

    @Override
    public void start() {
        super.start();
//        currStage = shootingStages.MOVING;
    }


    @SuppressLint("DefaultLocale")
    @Override
    public void loop() {
        super.loop();
        controlAnglePointMovement();
        controlShootingMovement();
        controlJoystickMovement();
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
//        if(gamepad1.dpad_right) {
//            anglePoint.x += 6;
//        }
//        if(gamepad1.dpad_left) {
//            anglePoint.x -= 6;
//        }
//        if(gamepad1.dpad_up) {
//            anglePoint.y += 6;
//        }
//        if(gamepad1.dpad_down) {
//            anglePoint.y -= 6;
//        }
//
//        if(gamepad1.y) {
//            anglePointControlled = !anglePointControlled;
//        }
//
//        if(anglePointControlled) {
//            PPController.movementResult result = PPController.pointPointTurn(anglePoint, 0.8, Math.toRadians(40));
//            telemetry.addLine(String.format("movementR: %.2f", Math.toDegrees(result.turnDelta_rad)));
//        }


        if(gamepad1.left_trigger > 0.7) {
            turnToGoal = true;
        }

        if(turnToGoal) {
            PPController.movementResult result = PPController.pointAngle(Math.toRadians(90), 0.8, Math.toRadians(45));
            telemetry.addLine(String.format("movementR: %.2f", Math.toDegrees(result.turnDelta_rad)));

            if(result.turnDelta_rad < Math.toRadians(2)) {
                turnToGoal = false;
            }
        }
    }

    public void controlShootingMovement() {
        if (gamepad1.right_trigger > 0.7) {
            goToShootingPoint = true;
        }

//        if (goToShootingPoint) {
//            if(currStage == shootingStages.MOVING) {
//                boolean done = goToPosition(0, 0, 0.8, Math.toRadians(90),
//                        0.6, Math.toRadians(40), 0.6, 2, true).withinBounds;
//
//                if (done) {
//                    DriveTrain.stopMovement();
//                    currStage = shootingStages.TURNING;
//                }
//            }
//
//            if(currStage == shootingStages.TURNING) {
//                PPController.movementResult result = PPController.pointAngle(Math.toRadians(90), 0.8, Math.toRadians(45));
//
//                if(result.turnDelta_rad < Math.toRadians(2)) {
//                    DriveTrain.stopMovement();
//                    goToShootingPoint = false;
//                    currStage = shootingStages.MOVING;
//                }
//            }
//        }
//        }


        if(goToShootingPoint) {
            if(stageFinished) {
                initProgVars();
            }
            ArrayList<CurvePoint> allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(stageStartPose.x, stageStartPose.y, 0, 0, 0, 0, 0, 0));
            allPoints.add(new CurvePoint(0, 0, 0.8, 0.8,  10, 15, Math.toRadians(30), 0.6));
            boolean done  = betterFollowCurve(allPoints, Math.toRadians(90), null, true, Math.toRadians(90));
            if(done) {
                DriveTrain.stopMovement();
                goToShootingPoint = false;
                stageFinished = true;
            }
        }
    }

    private boolean stageFinished = true;
    private Pose stageStartPose;
    private void initProgVars() {
        stageStartPose = Odometry.currentPosition;
        stageFinished = false;
    }
//

    public void telemetryVars() {
//        telemetry.addLine("anglePoint: " + anglePoint.toString());
//        telemetry.addLine("headingControlled: " + anglePointControlled);
        telemetry.addLine("turnToGoal: " + turnToGoal);
        telemetry.addLine("goToShootingPoint: " + goToShootingPoint);
        telemetry.addLine("90diff: " + Math.toDegrees(MathUtil.angleWrap(Math.toRadians(90) - Odometry.currentPosition.heading)));
    }


}
