// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.PrimoShuffleboard;
import frc.lib.PrimoTab;
import frc.robot.SwerveModule;
import frc.robot.subsystems.Swerve;

public class TuneDriveMotor extends CommandBase {
  /** Creates a new TuneDriveMotor. */
  private Swerve swerve;

  private NetworkTableEntry moduleNum, speedSetpoint, currentSpeed; 
  private SwerveModule module;

  private PrimoTab tab;

  public TuneDriveMotor(Swerve swerve) {
    this.swerve = swerve;
    addRequirements(swerve);

    tab = PrimoShuffleboard.getInstance().getPrimoTab("Drive Motor Tuning");
    moduleNum = tab.addEntry("Module Num");
    speedSetpoint = tab.addEntry("Speed Setpoint");
    currentSpeed = tab.addEntry("Current Speed");

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    module = swerve.getModule(moduleNum.getNumber(0).intValue());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    module.setDesiredState(new SwerveModuleState(speedSetpoint.getDouble(0), Rotation2d.fromDegrees(0)), false);
    currentSpeed.setNumber(module.getState().speedMetersPerSecond);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
