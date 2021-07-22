package org.firstinspires.ftc.teamcode.control.path;

import android.annotation.SuppressLint;

import org.firstinspires.ftc.teamcode.util.Point;
import org.firstinspires.ftc.teamcode.util.MathUtil;

import java.util.Arrays;
import java.util.LinkedList;

public class PathPoints {
    public enum types {
        lateTurn, onlyTurn, stop, locked, onlyFunctions;

        public boolean isLocked() {
            return ordinal() < 4;
        }
    }

    public static class BasePathPoint extends Point {
        public double followDistance;
        public LinkedList<Function> functions;
        public String signature;

        public Point lateTurnPoint;
        public Boolean isOnlyTurn;
        public Boolean isStop;
        public Double lockedHeading;
        public Boolean isOnlyFuncs;


        public BasePathPoint(String signature, double x, double y, double followDistance, Function... functions) {
            super(x, y);
            this.followDistance = followDistance;
            this.functions = new LinkedList<>();
            this.functions.addAll(Arrays.asList(functions));
            this.signature = signature;
        }

        public BasePathPoint(BasePathPoint b) { // copys metadata with list
            this(b.signature, b.x, b.y, b.followDistance);
            functions = b.functions;

            lateTurnPoint = b.lateTurnPoint;
            isOnlyTurn = b.isOnlyTurn;
            isStop = b.isStop;
            lockedHeading = b.lockedHeading;
            isOnlyFuncs = b.isOnlyFuncs;
        }

        private static Function[] linkedToPrim(LinkedList<Function> funcs) {
            Object[] oArr = funcs.toArray();
            Function[] fArr = new Function[oArr.length];
            for(int i=0; i<oArr.length; i++) {
                fArr[i] = (Function) oArr[i];
            }
            return fArr;
        }

        public Object[] getTypeList() {
            return new Object[] {lateTurnPoint, isOnlyTurn, isStop,  lockedHeading, isOnlyFuncs};
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

    public static class LateTurnPathPoint extends ControlledPathPoint {
        public LateTurnPathPoint(String signature, double x, double y, double heading, double followDistance, Point turnPoint, Function... functions) {
            super(signature, x, y, heading, followDistance, functions);
            lateTurnPoint = turnPoint;
        }
    }

    public static class OnlyTurnPathPoint extends ControlledPathPoint {
        public OnlyTurnPathPoint(String signature, double heading, Function... functions) {
            super(signature,0,0,heading,0,functions);
            isOnlyTurn = true;
        }
    }

    public static class StopPathPoint extends ControlledPathPoint {
        public StopPathPoint(String signature, double x, double y, double heading, double followDistance, Function... functions) {
            super(signature, x, y, heading, followDistance, functions);
            isStop = true;
        }
    }

    public static class ControlledPathPoint extends SimplePathPoint {
        public ControlledPathPoint(String signature, double x, double y, double heading, double followDistance, Function... functions) {
            super(signature, x, y, followDistance, functions);
            lockedHeading = MathUtil.unwrap(heading);
        }
    }

    public static class OnlyFunctionsPathPoint extends SimplePathPoint {
        public OnlyFunctionsPathPoint(String signature, Function... functions) {
            super(signature,0,0,0,functions);
            isOnlyFuncs = true;
        }
    }

    public static class SimplePathPoint extends BasePathPoint {
        public SimplePathPoint(String signature, double x, double y, double followDistance, Function... functions) {
            super(signature, x, y, followDistance, functions);
        }
    }
}