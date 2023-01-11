// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOptions;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.SwerveModule;
import frc.robot.subsystems.Swerve;

public class TuneDriveMotor extends CommandBase {
  /** Creates a new TuneDriveMotor. */
  private Swerve swerve;

  private SwerveModule module;

  private IntegerSubscriber moduleNum;
  private DoubleSubscriber setpointSubscriber;

  private DoublePublisher currentSpeed;

  public TuneDriveMotor(Swerve swerve) {
    this.swerve = swerve;
    addRequirements(swerve);

    NetworkTableInstance inst = NetworkTableInstance.getDefault();

    NetworkTable table = inst.getTable("Shuffleboard").getSubTable("Drive Motor Tuning");

    moduleNum = table.getIntegerTopic("Module Num").subscribe(0);


    setpointSubscriber = table.getDoubleTopic("Speed Setpoint").subscribe(0);
    currentSpeed = table.getDoubleTopic("Current speed").publish();

    addRequirements(swerve);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Long l = moduleNum.get();
    module = swerve.getModule(l.intValue());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    module.setDesiredState(new SwerveModuleState(setpointSubscriber.get(), Rotation2d.fromDegrees(0)), false);
    currentSpeed.accept(module.getState().speedMetersPerSecond);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    module.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)), false); 
    moduleNum.close();
    currentSpeed.close();
    setpointSubscriber.close();   
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
