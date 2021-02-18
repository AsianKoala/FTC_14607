package org.firstinspires.ftc.teamcode.control;

import org.firstinspires.ftc.teamcode.movement.CurvePoint;
import org.firstinspires.ftc.teamcode.util.MathUtil;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.movement.Odometry.currentPosition;

public class Results {
    public static class baseResult {
        public boolean done;
    }

    public static class movementResult extends baseResult {
        public double turnDelta_rad;
        public movementResult(double targetX, double targetY, double targetAngle, double moveThresh, double angleThresh){
            turnDelta_rad = MathUtil.angleWrap(targetAngle - currentPosition.heading);
            done = Math.hypot(targetX - currentPosition.x, targetY - currentPosition.y) < moveThresh && MathUtil.subtractAngleBool(currentPosition.heading, targetAngle, angleThresh);
        }
    }

    public static class simpleResult extends baseResult {
        public simpleResult(double current, double target, double thresh)  {
            done = Math.abs(current - target) < thresh;
        }
    }

    public static class complexResult extends baseResult {
        public ArrayList<Functions.function> functions;
        public complexResult(ArrayList<Functions.function> functions) {
            this.functions = functions;
            boolean isDone = true;
            for(Functions.function f : functions) {
                if(!f.result().done)
                    isDone = false;
            }
            done = isDone;
        }
    }

    static class test {
        public static void main(String[] args) {
            ArrayList<CurvePoint> allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint.complexCurvePoint(0, 0, 0, 0, 0, 0, 0, 0, new Functions.complexFunction() {
                @Override
                public complexResult runComplexFunctions() {
                    ArrayList<Functions.function> allRunnableFunctions = new complexListBuilder()
                            .add(new Functions.hardwareFunction() {
                                @Override
                                public simpleResult runHardware() {
                                    return null;
                                }

                                @Override
                                public baseResult result() {
                                    return null;
                                }

                                @Override
                                public boolean startCondition() {
                                    return false;
                                }
                            })
                    return null;
                }

                @Override
                public baseResult result() {
                    return runComplexFunctions();
                }

                @Override
                public boolean startCondition() {
                    return Math.hypot(currentPosition.x, currentPosition.y) < 5;
                }
            }));
        }
    }

    public static class complexListBuilder {
        private ArrayList<Functions.function> functions;

        public complexListBuilder add(Functions.function function) {
            functions.add(function);
            return this;
        }

        public ArrayList<Functions.function> build() {
            return functions;
        }
    }
}
