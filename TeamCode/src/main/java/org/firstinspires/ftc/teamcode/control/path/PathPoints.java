package org.firstinspires.ftc.teamcode.control.path;

import org.firstinspires.ftc.teamcode.util.Point;

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

        public Point lateTurnPoint;
        public Boolean onlyTurnHeading;
        public Boolean isStop;
        public Double lockedHeading;
        public Boolean isOnlyFuncs;


        public BasePathPoint(double x, double y, double followDistance, Function... functions) {
            super(x, y);
            this.followDistance = followDistance;
            this.functions = new LinkedList<>();
            this.functions.addAll(Arrays.asList(functions));
        }

        public BasePathPoint(BasePathPoint b) { // copys metadata with list
            this(b.x, b.y, b.followDistance);
            functions = b.functions;

            lateTurnPoint = b.lateTurnPoint;
            onlyTurnHeading = b.onlyTurnHeading;
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
            return new Object[] {lateTurnPoint, onlyTurnHeading, isStop,  lockedHeading, isOnlyFuncs};
        }


    }

    public static class LateTurnPathPoint extends LockedPathPoint {
        public LateTurnPathPoint(double x, double y, double heading, double followDistance, Point turnPoint, Function... functions) {
            super(x, y, heading, followDistance, functions);
            lateTurnPoint = turnPoint;
        }
    }

    public static class OnlyTurnPathPoint extends LockedPathPoint {
        public OnlyTurnPathPoint(double heading, Function... functions) {
            super(0,0,heading,0,functions);
            onlyTurnHeading = true;
        }
    }

    public static class StopPathPoint extends LockedPathPoint {
        public StopPathPoint(double x, double y, double heading, double followDistance, Function... functions) {
            super(x, y, heading, followDistance, functions);
            isStop = true;
        }
    }

    public static class LockedPathPoint extends SimplePathPoint {
        public LockedPathPoint(double x, double y, double heading, double followDistance, Function... functions) {
            super(x, y, followDistance, functions);
            lockedHeading = heading;
        }
    }

    public static class OnlyFunctionsPathPoint extends SimplePathPoint {
        public OnlyFunctionsPathPoint(Function... functions) {
            super(0,0,0,functions);
            isOnlyFuncs = true;
        }
    }

    public static class SimplePathPoint extends BasePathPoint {
        public SimplePathPoint(double x, double y, double followDistance, Function... functions) {
            super(x, y, followDistance, functions);
        }
    }
}