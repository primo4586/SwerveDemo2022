// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import java.util.List;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.paths.Path;
import frc.lib.paths.PathSelector;
import frc.robot.autos.FollowTrajectory;
import frc.robot.subsystems.Swerve;

/** Add your docs here. */
public class AutoContainer {

    private Swerve swerveDrive;
    private PathSelector pathSelector;

    public AutoContainer(Swerve swerveDrive) {
        this.swerveDrive = swerveDrive;
        pathSelector = new PathSelector(Paths.pathSelectorEntries, "Pathfollowing");
    }

    public Command getSelectedCommand() {
        return new FollowTrajectory(pathSelector.getPath(), swerveDrive);
    }
  
    public static final class Paths {
        
        public static final Rotation2d zeroRotation = new Rotation2d();
        public static final Pose2d zeroPose = new Pose2d(0, 0, zeroRotation);
        
        public static final Path oneMeterForward = new Path(zeroPose, new Pose2d(1, 0, new Rotation2d()));
        public static final Path oneMeterBackward = new Path(zeroPose, new Pose2d(-1, 0, new Rotation2d()));

        public static final Path threeMeterSCurvePath = new Path(zeroPose, List.of(new Translation2d(1, 1), new Translation2d(2, -1)), new Pose2d(3, 0, zeroRotation));

        public static final Map<String, Path> pathSelectorEntries = Map.ofEntries(
            Map.entry("One Meter Forward", oneMeterForward),
            Map.entry("One Meter Backward", oneMeterBackward),
            Map.entry("Three Meter S Curve", threeMeterSCurvePath)
        );

       
    }
}
