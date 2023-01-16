package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.SwerveModule;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj.Joystick;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public class TeleopSwerve extends CommandBase {

    private double rotation;
    private Translation2d translation;
    private boolean fieldRelative;
    private boolean openLoop;
    
    private Swerve s_Swerve;
    private CommandXboxController controller;
    private int translationAxis;
    private int strafeAxis;
    private int rotationAxis;

    private BooleanSupplier slowMode;

    /**
     * Driver control
     */
    public TeleopSwerve(Swerve s_Swerve, CommandXboxController controller, int translationAxis, int strafeAxis, int rotationAxis, boolean fieldRelative, boolean openLoop, BooleanSupplier slowMode) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.controller = controller;
        this.translationAxis = translationAxis;
        this.strafeAxis = strafeAxis;
        this.rotationAxis = rotationAxis;
        this.fieldRelative = fieldRelative;
        this.openLoop = openLoop;
        this.slowMode = slowMode ;
    }

    @Override
    public void initialize() {
        //s_Swerve.stopModules();
        System.out.println("vroom vroom");
    }

    @Override
    public void execute() {
        double yAxis = -controller.getRawAxis(translationAxis);
        double xAxis = -controller.getRawAxis(strafeAxis);
        double rAxis = -controller.getRawAxis(rotationAxis);
        
        /* Deadbands */
        yAxis = (Math.abs(yAxis) < Constants.stickDeadband) ? 0 : yAxis;
        xAxis = (Math.abs(xAxis) < Constants.stickDeadband) ? 0 : xAxis;
        rAxis = (Math.abs(rAxis) < Constants.stickDeadband) ? 0 : rAxis;
        
        
        if(!slowMode.getAsBoolean()){
            translation = new Translation2d(yAxis, xAxis).times(Constants.SwerveConstants.maxSpeed);
            rotation = rAxis * Constants.SwerveConstants.maxAngularVelocity;
            s_Swerve.drive(translation, rotation, fieldRelative, openLoop);
        }
        else{
            translation = new Translation2d(yAxis, xAxis).times(Constants.SwerveConstants.slowModeSpeed);
            rotation = rAxis * Constants.SwerveConstants.slowModeAngularVelocity;
            s_Swerve.drive(translation, rotation, fieldRelative, openLoop);
        }
        

    }
    
   @Override
   public void end(boolean interrupted) {
    // s_Swerve.stopModules();
}

   
}
