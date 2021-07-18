package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.control.path.Path;
import org.firstinspires.ftc.teamcode.control.path.builders.PathBuilder;
import org.firstinspires.ftc.teamcode.control.system.BaseAuto;

import java.util.LinkedList;

import static org.firstinspires.ftc.teamcode.control.path.PathPoints.*;

@Autonomous
public class AzusaAuto extends BaseAuto {
    @Override
    public LinkedList<Path> pathList() {
        LinkedList<Path> returnList = new LinkedList<>();
        PathBuilder leftBall = new PathBuilder("left ball")
                .addPoint(new BasePathPoint("start", 88, 150, 0))
                .addPoint(new BasePathPoint("intermed 1", 45, 135, 35))
                .addPoint(new BasePathPoint("left-left", 30, 90, 35))
                .addPoint(new BasePathPoint("intermed 2", 45, 45, 35))
                .addPoint(new BasePathPoint("left-bottom", 88, 30, 35))
                .addPoint(new BasePathPoint("intermed 3", 130, 43, 35))
                .addPoint(new BasePathPoint("left-right", 150, 90, 35))
                .addPoint(new BasePathPoint("intermed 4", 130, 135, 35))
                .addPoint(new BasePathPoint("start2", 88, 150, 35));
        returnList.add(leftBall.build());
        return returnList;
    }
}
