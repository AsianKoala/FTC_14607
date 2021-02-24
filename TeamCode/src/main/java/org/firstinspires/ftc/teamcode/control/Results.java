package org.firstinspires.ftc.teamcode.main.control;

import org.firstinspires.ftc.teamcode.main.util.Util;

import static org.firstinspires.ftc.teamcode.main.movement.Odometry.currentPosition;

public class Results {
    public static class baseResult {
        public boolean done;
    }

    public static class movementResult extends baseResult {

    }

    public static class translationResult extends movementResult {
        public translationResult(double targetX, double targetY, double thresh) {
            done = Math.hypot(currentPosition.x - targetX, currentPosition.y - targetY) < thresh;
        }
    }

    public static class turnResult extends movementResult {
        public double absDeltaAngle;
        public turnResult(double targetAngle, double thresh) {
            done = Util.subtractAngleBool(currentPosition.heading, targetAngle, thresh);
            absDeltaAngle = Math.abs(Util.angleWrap(targetAngle - currentPosition.heading));
        }
    }

    public static class goToPositionResult extends movementResult {
        public goToPositionResult(translationResult mResult, turnResult tResult) {
            done = mResult.done && tResult.done;
        }
    }

    public static class diffResult extends baseResult {
        public diffResult(double current, double target, double thresh) {
            done = Math.abs(current - target) < thresh;
        }
    }



}
