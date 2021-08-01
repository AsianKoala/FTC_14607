package org.firstinspires.ftc.teamcode.control.path;

import java.util.ArrayList;
import java.util.LinkedList;

import org.firstinspires.ftc.teamcode.control.system.Azusa;
import org.firstinspires.ftc.teamcode.control.path.builders.*;
import org.firstinspires.ftc.teamcode.control.controllers.PurePursuitController;
import org.firstinspires.ftc.teamcode.util.MathUtil;

public class Path extends LinkedList<PathPoint> {
    // target is always getFirst(), curr is copied
    public PathPoint curr;
    public boolean isPurePursuit;
//    public boolean copi = true; //TODO i forgot what this does go to sim and check it out
    public ArrayList<PathPoint> initialPoints;
    public String name;
    private boolean firstFinish;

    public Path(Path path) {
        for (PathPoint pathPoint : path) add(new PathPoint(pathPoint));

        initialPoints = new ArrayList<>();
        for(PathPoint pathPoint : this) initialPoints.add(new PathPoint(pathPoint));

        isPurePursuit = true;
        curr = new PathPoint(getFirst());
        removeFirst();

        name = path.name;
        firstFinish = false;
    }

    public Path(String name) {
        this.name = name;
    }

    public Path(PathPoint target, String name) {
        this(new PathBuilder(name).addPoint(target).build());
        isPurePursuit = false;
    }

    public Path() {
        isPurePursuit = true;
    }

    public void follow(Azusa azusa) {
        PathPoint target = getFirst();
        if(!isPurePursuit) {
            PurePursuitController.goToPosition(azusa, target, curr);
        } else {
            boolean skip;

            if (target instanceof OnlyTurnPoint) {
                skip = MathUtil.angleThresh(azusa.currPose.h, ((OnlyTurnPoint) target).h);
            } else if(target instanceof StopPathPoint){
                skip = azusa.currPose.distance(target) < 2;
            } else {
                skip = azusa.currPose.distance(target) < target.followDistance;
                if(size()>1 && get(1) instanceof StopPathPoint) {
                    skip = azusa.currPose.distance(target) < 10;
                }
            }

//            if(!target.functions.isEmpty()) {
//                target.functions.removeIf(f -> f.cond() && f.func());
//                skip = skip && target.functions.size() == 0;
//            } // TODO

            if (skip) {
                curr = new PathPoint(target); // swap old target to curr start
                removeFirst();
            }

            if(isEmpty()) {
                firstFinish = true;
                return;
            }

            if(target instanceof StopPathPoint && azusa.currPose.distance(target) < target.followDistance) {
                PurePursuitController.goToPosition(azusa, target, curr);
            } else {
                PurePursuitController.followPath(azusa, curr, target);
            }
        }
    }

    @Override
    public String toString() {
        StringBuilder s = new StringBuilder("Path Name: " + name);
        String newLine = System.getProperty("line.separator");
        for(PathPoint p : initialPoints) {
            s.append(newLine).append("\t");

            if(curr.equals(p))
                s.append("curr:");
            else
                s.append("\t");
            s.append("\t");
            s.append(p);
        }
        s.append(newLine);
        return s.toString();
    }

    public boolean finished() {
        if(firstFinish) {
            firstFinish = false;
            return true;
        }
        return false;
    }

}