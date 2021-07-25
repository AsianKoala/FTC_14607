package org.firstinspires.ftc.teamcode.control.path;

import android.annotation.SuppressLint;

import org.firstinspires.ftc.teamcode.util.Point;
import org.firstinspires.ftc.teamcode.util.MathUtil;

import java.util.Arrays;
import java.util.LinkedList;

public class PathPoints {

    public static class BasePathPoint extends Point {
        public double followDistance;
        public LinkedList<Function> functions;
        public String signature;

        public Point lateTurnPoint;
        public boolean isOnlyTurn;
        public boolean isStop;
        public boolean isOnlyFuncs;
        public boolean isLocked;
        public double lockedHeading;

        public BasePathPoint(String signature, double x, double y, double followDistance, Function... functions) {
            super(x, y);
            this.followDistance = followDistance;
            this.functions = new LinkedList<>();
            this.functions.addAll(Arrays.asList(functions));
            this.signature = signature;

            isOnlyTurn = false;
            isStop = false;
            isOnlyFuncs = false;
            isLocked = false;
        }

        public BasePathPoint(BasePathPoint b) {
            this(b.signature, b.x, b.y, b.followDistance);
            functions = b.functions;

            lateTurnPoint = b.lateTurnPoint;
            isOnlyTurn = b.isOnlyTurn;
            isStop = b.isStop;
            isOnlyFuncs = b.isOnlyFuncs;
            isLocked = b.isLocked;
            lockedHeading = b.lockedHeading;
        }

        @SuppressLint("DefaultLocale")
        @Override
        public String toString() {
            return String.format("%s, %.1f, %.1f, %.1f", signature, x, y, followDistance);
        }

        public boolean equals(BasePathPoint b) {
            return x==b.x && y==b.y;
        }
    }

    public static class ControlledPathPoint extends BasePathPoint {
        public ControlledPathPoint(String signature, double x, double y, double heading, double followDistance, Function... functions) {
            super(signature, x, y, followDistance, functions);
            isLocked = true;
            lockedHeading = heading;
        }
    }

    public static class StopPathPoint extends ControlledPathPoint {
        public StopPathPoint(String signature, double x, double y, double heading, double followDistance, Function... functions) {
            super(signature, x, y, heading, followDistance, functions);
            isStop = true;
        }
    }

//    public static class LateTurnPathPoint extends ControlledPathPoint {
//        public LateTurnPathPoint(String signature, double x, double y, double heading, double followDistance, Point turnPoint, Function... functions) {
//            super(signature, x, y, heading, followDistance, functions);
//            lateTurnPoint = turnPoint;
//        }
//    }
//
//    public static class OnlyTurnPathPoint extends ControlledPathPoint {
//        public OnlyTurnPathPoint(String signature, double heading, Function... functions) {
//            super(signature,0,0,heading,0,functions);
//            isOnlyTurn = true;
//        }
//    }
//
//    public static class OnlyFunctionsPathPoint extends SimplePathPoint {
//        public OnlyFunctionsPathPoint(String signature, Function... functions) {
//            super(signature,0,0,0,functions);
//            isOnlyFuncs = true;
//        }
//    }
}