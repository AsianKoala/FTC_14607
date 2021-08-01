package org.firstinspires.ftc.teamcode.control.system;

import org.firstinspires.ftc.teamcode.util.Pose;

// typically we want everything to be a function so its easier for interrupts/listeners
public class Functions {
    public interface Function {}

    public interface SimpleFunction extends Function {
        void run(Azusa azusa);
    }

    public interface LoopFunction extends Function {
        boolean run(Azusa azusa);
    }

    public interface InterruptFunction extends Function {
        boolean run(Azusa azusa);
    }

    public class TimeFunction {
        public long targetTime;
        public SimpleFunction func;

        public TimeFunction(long dt, SimpleFunction func) {
            targetTime = dt + System.currentTimeMillis();
            this.func = func;
        }
    }

    public class PoseFunction {
        public Pose targetPose;
        public SimpleFunction func;

        public PoseFunction(Pose targetPose, SimpleFunction func) {
            this.targetPose = targetPose;
            this.func = func;
        }
    }
}