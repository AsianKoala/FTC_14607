package org.firstinspires.ftc.teamcode.control;

public abstract class Auto extends Robot {

    @Override
    public void init() {
        super.init();
    }

    @Override
    public void init_loop() {
        super.init_loop();
    }

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void loop() {
        super.loop();
        autoStateMachine();
    }

    public abstract void autoStateMachine();
}
