package frc.lib.paths;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

public class Path {

    private Pose2d startPose;
    private Pose2d endPose;
    private List<Translation2d> interiorPoints;

    public Path(Pose2d startPose, List<Translation2d> interiorPoints, Pose2d endPose) {
        this.startPose = startPose;
        this.interiorPoints = interiorPoints;
        this.endPose = endPose;
    }

    public Path(Pose2d startPose, Pose2d endPose) {
        this(startPose, new ArrayList<>(), endPose);
    }

    public Trajectory generateAsTrajectory(TrajectoryConfig config) {
        return TrajectoryGenerator.generateTrajectory(startPose, interiorPoints, endPose, config);
    }

    public Pose2d getStartPose() {
        return startPose;
    }

    public List<Translation2d> getInteriorPoints() {
        return interiorPoints;
    }

    public Pose2d getEndPose() {
        return endPose;
    }

}