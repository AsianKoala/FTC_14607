package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.control.path.Path;
import org.firstinspires.ftc.teamcode.control.path.builders.PathBuilder;
import org.firstinspires.ftc.teamcode.control.system.BaseAuto;
import org.firstinspires.ftc.teamcode.util.Pose;

import java.util.LinkedList;

import static org.firstinspires.ftc.teamcode.control.path.PathPoints.*;

@Autonomous
public class AzusaAuto extends BaseAuto {
    @Override
    public Pose startPose() {
        return new Pose(85, 9, Math.PI);
    }

    @Override
    public LinkedList<Path> pathList() {
        LinkedList<Path> returnList = new LinkedList<>();
        PathBuilder lCurve = new PathBuilder("lCurve")
                .addPoint(new BasePathPoint("start", 85,9,0))
                .addPoint(new BasePathPoint("", 61,15,14))
                .addPoint(new BasePathPoint("",37, 24,14))
                .addPoint(new BasePathPoint("", 24, 36, 14))
                .addPoint(new BasePathPoint("", 15, 50,14))
                .addPoint(new BasePathPoint("", 12, 64, 14))
                .addPoint(new BasePathPoint("", 10, 80, 14))
                .addPoint(new BasePathPoint("", 10, 104, 14));

        returnList.add(lCurve.build());
        return returnList;
    }
}