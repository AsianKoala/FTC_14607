package org.firstinspires.ftc.teamcode.control;

import org.firstinspires.ftc.teamcode.hardware.Hardware;

public class Functions {
    public interface function {}

    public interface simpleHardwareFunction extends function {
        Result.simpleResult simpleHardwareResult(Hardware hardware);
    }


    public static simpleHardwareFunction TURN_ON = new simpleHardwareFunction() {
        @Override
        public Result.simpleResult simpleHardwareResult(Hardware hardware) {
            hardware.turnOn();
            return new Result.simpleResult();
        }
    };

    public static simpleHardwareFunction TURN_OFF = new simpleHardwareFunction() {
        @Override
        public Result.simpleResult simpleHardwareResult(Hardware hardware) {
            hardware.turnOff();
            return new Result.simpleResult();
        }
    };

    public static simpleHardwareFunction REVERSE = new simpleHardwareFunction() {
        @Override
        public Result.simpleResult simpleHardwareResult(Hardware hardware) {
            hardware.reverse();
            return new Result.simpleResult();
        }
    };




}


