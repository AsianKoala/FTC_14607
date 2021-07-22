package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.control.path.Path;
import org.firstinspires.ftc.teamcode.control.path.PathPoints;
import org.firstinspires.ftc.teamcode.control.path.builders.PathBuilder;
import org.firstinspires.ftc.teamcode.control.system.Robot;
import org.firstinspires.ftc.teamcode.util.Debuggable;
import org.firstinspires.ftc.teamcode.util.Pose;

@Debuggable
@TeleOp
public class AzusaDebug extends Robot {

    @Override
    public Pose startPose() {
        return new Pose(0,0,0);
    }

    @Override
    public Path path() {
        PathBuilder lCurve = new PathBuilder("lCurve")
                .addPoint(new PathPoints.BasePathPoint("start", 0,0,0))
                .addPoint(new PathPoints.BasePathPoint("first target", 0,15,14))
                .addPoint(new PathPoints.BasePathPoint("",5, 23,14))
                .addPoint(new PathPoints.BasePathPoint("", 9, 27, 14))
                .addPoint(new PathPoints.BasePathPoint("", 24, 36,14))
                .addPoint(new PathPoints.BasePathPoint("", 40, 40, 14))
                .addPoint(new PathPoints.StopPathPoint("stop", 60, 40, 0,14));
        return lCurve.build();
    }
}
