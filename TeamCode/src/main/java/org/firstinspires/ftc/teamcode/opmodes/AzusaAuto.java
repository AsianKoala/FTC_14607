package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.control.path.Path;
import org.firstinspires.ftc.teamcode.control.path.PathPoint;
import org.firstinspires.ftc.teamcode.control.path.StopPathPoint;
import org.firstinspires.ftc.teamcode.control.path.builders.PathBuilder;
import org.firstinspires.ftc.teamcode.control.system.BaseAuto;
import org.firstinspires.ftc.teamcode.util.Pose;

@Autonomous
public class AzusaAuto extends BaseAuto {
    @Override
    public Pose startPose() {
        return new Pose(-64, -64, Math.PI / 2);
    }

    @Override
    public Path path() {
        PathBuilder exp = new PathBuilder("lCurve exp")
                .addPoint(new PathPoint("start", -64,-64,0))
                .addPoint(new PathPoint("fw", -64, -34, 14))
                .addPoint(new PathPoint("joint 1", -46, -10, 14))
                .addPoint(new PathPoint("joint 2", -16, 8, 14))
                .addPoint(new PathPoint("track 1", 16, 12, 14))
                .addPoint(new StopPathPoint("track 2", 56, 12, 0, 14));
        return exp.build();
    }
}