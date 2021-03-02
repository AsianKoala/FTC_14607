package org.firstinspires.ftc.teamcode.opmodes.auto;

import org.firstinspires.ftc.teamcode.control.controllers.Function;
import org.firstinspires.ftc.teamcode.control.controllers.Path;
import org.firstinspires.ftc.teamcode.control.controllers.PathBuilder;
import org.firstinspires.ftc.teamcode.control.controllers.PathPoint;
import org.firstinspires.ftc.teamcode.control.controllers.PathPointBuilder;

public class auto1 extends BaseAuto {
    @Override
    public Path path() {
        Function turnOnIntake = (cond) -> {
            robot.update(); // IDK BRUH THIS IS JUST AN EXAMPLE ANYWAY
            return true;
        };

        Function isHalfwayDone = (cond) -> cond && robot.isFollowing;

        return new PathBuilder()
                .addPoint(new PathPoint(startPose(), 0))
                .addPoint(new PathPoint(10, 10, Math.toRadians(90), 4))
                .addPoint(new PathPoint(20, 20, Math.toRadians(45), 4))
                .addPoint(new PathPointBuilder(30, 30, Math.toRadians(0), 4)
                        .addFunc(null)
                        .addFunc(turnOnIntake)
                        .addFunc(isHalfwayDone)
                        .build())
                .build();
    }
}
