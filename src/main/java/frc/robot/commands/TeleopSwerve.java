package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.SwerveModule;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class TeleopSwerve extends CommandBase {

    private double rotation;
    private Translation2d translation;
    private boolean fieldRelative;
    private boolean openLoop;
    
    private Swerve s_Swerve;
    private Joystick controller;
    private int translationAxis;
    private int strafeAxis;
    private int rotationAxis;

    /**
     * Driver control
     */
    public TeleopSwerve(Swerve s_Swerve, Joystick controller, int translationAxis, int strafeAxis, int rotationAxis, boolean fieldRelative, boolean openLoop) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.controller = controller;
        this.translationAxis = translationAxis;
        this.strafeAxis = strafeAxis;
        this.rotationAxis = rotationAxis;
        this.fieldRelative = fieldRelative;
        this.openLoop = openLoop;
    }

    @Override
    public void initialize() {
        s_Swerve.stopModules();
        System.out.println("broom vroom");
    }

    @Override
    public void execute() {
        double yAxis = -controller.getRawAxis(translationAxis);
        double xAxis = -controller.getRawAxis(strafeAxis);
        double rAxis = -controller.getRawAxis(rotationAxis);
        
        /* Deadbands */ 
        yAxis = (Math.abs(yAxis) < Constants.stickDeadband) ? 0 : yAxis;
        // yAxis = 0;
        xAxis = 0;
        
        // xAxis = (Math.abs(xAxis) < Constants.stickDeadband) ? 0 : xAxis;
        rAxis = (Math.abs(rAxis) < Constants.stickDeadband) ? 0 : rAxis;

        translation = new Translation2d(yAxis, xAxis).times(Constants.SwerveConstants.maxSpeed);
        SmartDashboard.putNumber("Translation Y", translation.getY());
        SmartDashboard.putNumber("Translation X", translation.getX());

        rotation = rAxis * Constants.SwerveConstants.maxAngularVelocity;
        SmartDashboard.putNumber("Rotation ", rotation);

        s_Swerve.drive(translation, rotation, fieldRelative, openLoop);

        // SwerveModuleState empty = new SwerveModuleState(0, Rotation2d.fromDegrees(0));
        // SwerveModuleState[] moduleStates = new SwerveModuleState[]{new SwerveModuleState(0, Rotation2d.fromDegrees(45)), empty, empty, empty};
        // s_Swerve.setModuleStatesOpenLoop(moduleStates);
    }
    
   @Override
   public void end(boolean interrupted) {
    s_Swerve.stopModules();
}

   
}
