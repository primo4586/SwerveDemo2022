// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PathPlannerFollowCommand extends SequentialCommandGroup {
  /** Creates a new PathPlannerFollowCommand. */
  public PathPlannerFollowCommand(Swerve swerve) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    PathPlannerTrajectory trajectory = PathPlanner.loadPath("One Meter Forward", AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared);

    
    var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);


      System.out.println(trajectory.getInitialState().holonomicRotation);
    addCommands(
      new InstantCommand(() -> swerve.resetOdometryWithNewRotation(trajectory.getInitialPose(), trajectory.getInitialState().holonomicRotation), swerve),
      new PPSwerveControllerCommand(
        trajectory,
      () -> swerve.getPose(),
      SwerveConstants.swerveKinematics,
      new PIDController(AutoConstants.kPXController, 0, 0),
      new PIDController(AutoConstants.kPYController, 0, 0),
      thetaController,
      swerve::setModuleStates,
      swerve
      ));



  }
}
