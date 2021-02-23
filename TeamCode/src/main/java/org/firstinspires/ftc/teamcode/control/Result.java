package org.firstinspires.ftc.teamcode.control;

import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.Pose;

public class Result {

    public static class simpleResult extends Result {
        public AutoStartVars resultVars;

        public simpleResult() {
            resultVars.initialize();
        }
    }

    public static class followPathResult extends simpleResult {
        public boolean withinPositionalBounds;
        public boolean withinAngleBounds;

        public followPathResult(Pose targetPose, double positionThresh) {
            super();
            withinPositionalBounds = Math.hypot(resultVars.stageStartX - targetPose.x, resultVars.stageStartY - targetPose.y) < positionThresh;
            withinAngleBounds = Math.abs(MathUtil.angleWrap(targetPose.heading - resultVars.stageStartHeading)) < Math.toRadians(2);
        }
    }
}
