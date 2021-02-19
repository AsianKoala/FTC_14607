package org.firstinspires.ftc.teamcode.control;



public class Functions {
    public static abstract class function {
        public boolean startCondition() { return true; }
        public abstract void run();
    }

    public abstract static class hardwareFunction extends function {
        @Override
        public abstract void run();
    }

    public abstract static class headingControlledFunction extends function {
        public double heading;
        @Override
        public abstract void run();
    }

    public abstract static class conditionHardwareFunction extends hardwareFunction {
        @Override
        public abstract boolean startCondition();

        @Override
        public abstract void run();
    }

    public abstract static class finalTurnFunction extends headingControlledFunction {
        @Override
        public abstract void run();
    }

}


