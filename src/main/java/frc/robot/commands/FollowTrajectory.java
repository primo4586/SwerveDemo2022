// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.lib.paths.Path;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
/**
 * Follows a given WPILib trajectory with a swerve drive, basically a copy of {@link SwerveControllerCommand}, 
 * just more clean so it's more understandable
 */
public class FollowTrajectory extends CommandBase {

  private Trajectory trajectory;
  private HolonomicDriveController holomnicPoseController;  
  private Swerve swerveDrive;
  private Rotation2d targetAngle;
  private Timer timer;
  
  public FollowTrajectory(Trajectory trajectory, Swerve swerveDrive) {

    ProfiledPIDController thetaController = AutoConstants.thetaController;
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // Generates the target speed & speed of rotation values given for a target pose from the current position of the robot
    holomnicPoseController = new HolonomicDriveController(AutoConstants.xController.getController(Constants.loopPeriod), 
                                              AutoConstants.yController.getController(Constants.loopPeriod), 
                                              thetaController);
    holomnicPoseController.setEnabled(true);   // Enables the feedback part of it.                                      

    this.trajectory = trajectory;
    this.targetAngle = trajectory.getStates().get(trajectory.getStates().size() - 1).poseMeters.getRotation();

    this.swerveDrive = swerveDrive;
    this.timer = new Timer();
    addRequirements(swerveDrive);
  }

  public FollowTrajectory(Path path, Swerve swerveDrive) {
    this(path.generateAsTrajectory(AutoConstants.trajectoryConfig),swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Reset to initial position, since that's where realistically the robot should be at when the command starts.
    swerveDrive.resetOdometry(trajectory.getInitialPose());
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // We get the target state we should be at this point of time.
    Trajectory.State targetState = trajectory.sample(timer.get());

    // We calculate the required speed (linear velocity) and speed of rotation (angular velocity), to get to the target state we want to get to from our trajectory
    ChassisSpeeds targetSpeeds = holomnicPoseController.calculate(swerveDrive.getPose(), targetState, targetAngle);
    
    // Then convert it to module states -> the target speed and angle each module has to be in, in order to get to that target state, and apply it to the motors.
    SwerveModuleState[] desiredStates = SwerveConstants.swerveKinematics.toSwerveModuleStates(targetSpeeds);
    swerveDrive.setModuleStates(desiredStates);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    holomnicPoseController.setEnabled(false);                                         
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // We should be at the end position by the time the trajectory ends.
    // NOTE: alternatively, we could use HolomnicDriverController.atReference(), 
    // but in case it isn't able to get close enough, it's better to end it after the trajectory finishes to save time on autonomous.
    return timer.hasElapsed(trajectory.getTotalTimeSeconds());
  }
}
