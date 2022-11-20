package frc.robot.commands;

import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class TeleopSwerve extends CommandBase {

    private double rotation;
    private Translation2d translation;
    private boolean fieldRelative;
    
    private Swerve swerveDrive;
    private Joystick controller;
    private int translationAxis;
    private int strafeAxis;
    private int rotationAxis;
    /**
     * Driver control
     */
    public TeleopSwerve(Swerve swerveDrive, Joystick controller, int translationAxis, int strafeAxis, int rotationAxis, boolean fieldRelative) {
        this.swerveDrive = swerveDrive;
        addRequirements(swerveDrive);

        this.controller = controller;
        this.translationAxis = translationAxis;
        this.strafeAxis = strafeAxis;
        this.rotationAxis = rotationAxis;
        this.fieldRelative = fieldRelative;
    }

    @Override
    public void initialize() {
        swerveDrive.stopModules();
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

        translation = new Translation2d(yAxis, xAxis).times(Constants.SwerveConstants.MAX_SPEED);
        rotation = rAxis * SwerveConstants.MAX_ANGULAR_VELOCITY;
        swerveDrive.teleopDrive(translation, rotation, fieldRelative, true);
        // swerveDrive.teleopDrive(new Translation2d(0, 0.25 * yInput), 0, true, true);
        // swerveDrive.setModuleStateRotation(0, 135);
    }
}
