package frc.robot.commands;

import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class TeleopSwerve extends CommandBase {

    private double rotation;
    private Translation2d translation;
    private boolean fieldRelative;
    
    private Swerve swerveDrive;
    private Joystick controller;
    private int yAxis;
    private int xAxis;
    private int rotationAxis;

    private SlewRateLimiter xLimiter = Constants.X_LIMITER;
    private SlewRateLimiter yLimiter = Constants.Y_LIMITER;
    private SlewRateLimiter rotationLimiter = Constants.ROTATION_LIMITER;

    /**
     * Driver control
     */
    public TeleopSwerve(Swerve swerveDrive, Joystick controller, int yAxis, int xAxis, int rotationAxis, boolean fieldRelative) {
        this.swerveDrive = swerveDrive;
        addRequirements(swerveDrive);

        this.controller = controller;
        this.yAxis = yAxis;
        this.xAxis = xAxis;
        this.rotationAxis = rotationAxis;
        this.fieldRelative = fieldRelative;
    }

    @Override
    public void execute() {
        // TODO: Check if this invert is necessary 
        double yInput = -controller.getRawAxis(yAxis);
        double xInput = -controller.getRawAxis(xAxis);
        double rotationInput = -controller.getRawAxis(rotationAxis);
        
        /* Deadbands (If the joystick input is too low to be signifcant, protect the motors from trying to move really small distances) */
        yInput = (Math.abs(yInput) < Constants.stickDeadband) ? 0 : yInput;
        xInput = (Math.abs(xInput) < Constants.stickDeadband) ? 0 : xInput;
        rotationInput = (Math.abs(rotationInput) < Constants.stickDeadband) ? 0 : rotationInput;

        // Slew Rate Limiting (Limiting how big of a change in joystick inputs can happen every second)
        xInput = xLimiter.calculate(xInput);
        yInput = yLimiter.calculate(yInput);
        rotationInput = rotationLimiter.calculate(rotationInput);
        
        // Calculates the movement of the robot on the X,Y plane, X being forward/backwards, and Y being left/right.
        translation = new Translation2d(yInput, xInput).times(Constants.Swerve.maxSpeed);
        // Calculates the rotation movement.
        rotation = rotationInput * Constants.Swerve.maxAngularVelocity;

        swerveDrive.teleopDrive(translation, rotation, fieldRelative, true);
    }
}
