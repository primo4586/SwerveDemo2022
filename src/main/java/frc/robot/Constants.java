package frc.robot;


import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.PIDConfig;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    // Sets a minimum for how much someone needs to push the joystick in order to make the robot move.
    public static final double stickDeadband = 0.1;
    /**
     * SlewRateLimiters limit how big of changes can change in the input over an amount of time/second
     * @see {https://docs.wpilib.org/en/stable/docs/software/advanced-controls/filters/slew-rate-limiter.html}
     * TODO: Tune the slew rate limiter values / check if they are necessary and remove if not
     */
    public static final SlewRateLimiter X_LIMITER = new SlewRateLimiter(0.01);
    public static final SlewRateLimiter Y_LIMITER = new SlewRateLimiter(0.01);
    public static final SlewRateLimiter ROTATION_LIMITER = new SlewRateLimiter(0.1);


    public static final double loopPeriod = 0.02;

    // TODO: Update Physical Robot constants + ids, inverts and such.
    public static final class SwerveConstants {
        public static final int pigeonID = 1;
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        /* Drivetrain Constants (Physical Constants) */
        public static final double trackWidth = Units.inchesToMeters(21.73); 
        public static final double wheelBase = Units.inchesToMeters(21.73);
        public static final double wheelDiameter = Units.inchesToMeters(4); 
        public static final double wheelCircumference = wheelDiameter * Math.PI;

        /* Ramping - how fast the motors are/can move from 0 speed to full speed */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Gear Ratios */
        public static final double driveGearRatio = (8.14 / 1.0); //8.14:1
        public static final double angleGearRatio = (150 / 7) / 1.0; //150 / 7 : 1

        /* Swerve Kinematics, essentialy the physical "outline" of the robot and where modules are placed relative to the robot center */
        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* Angle Motor PID Values */
       public static final PIDConfig anglePID = new PIDConfig(0, 0, 0,0);

        /* Drive Motor PID Values */
        public static final PIDConfig drivePID = new PIDConfig(0, 0, 0, 0);

        /* Drive Motor Characterization Values */
        public static final double driveKs = (0 / 12); //divide by 12 to convert from volts to percent output for CTRE
        public static final double driveKv = (0 / 12);
        public static final double driveKa = (0 / 12);
        public static final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(SwerveConstants.driveKs,SwerveConstants.driveKv, SwerveConstants.driveKa);

        /* Swerve Profiling Values */
        public static final double maxSpeed = 4.5; //meters per second
        public static final double maxAngularVelocity = 11.5;

        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        /* Motor Inverts */
        public static final boolean driveMotorInvert = false;
        public static final boolean angleMotorInvert = false;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = false;

        /* Module Specific Constants */
        /** Front Left Module - Module 0 */
        public static final class FrontLeftModule {
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 1;
            public static final double angleOffset = 0;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /** Front Right Module - Module 1 */
        public static final class FrontRightModule {
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 2;
            public static final double angleOffset = 0;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /** Back Left Module - Module 2 */
        public static final class BackLeftModule {
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 3;
            public static final double angleOffset = 0;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /** Back Right Module - Module 3 */
        public static final class BackRightModule {
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 4;
            public static final double angleOffset = 0;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

    }

    public static final class AutoConstants {
        // Self-explanatory, Limits for speed (linear velocity) and speed of rotation (angular velocity)
        public static final double maxSpeedMetersPerSecond = 3;
        public static final double maxAccelerationMetersPerSecondSquared = 3;
        public static final double maxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double maxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        /**
         * These are SEPARATE from the Drive/Module PID Controllers, while the drive PID corrects errors from target velocity/setpoint for each motor,
         * These controllers are used to calculate the target velocities based off errors from the current robot pose to the target pose.
         * {@see FollowTrajectory} {@see HolonomicDriveController}
         */
        public static final PIDConfig xController = new PIDConfig(1, 0, 0);
        public static final PIDConfig yController = new PIDConfig(1, 0, 0);
        public static final PIDConfig thetaControllerPID = new PIDConfig(1, 0, 0);
    
        // Constraint for the motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints thetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                maxAngularSpeedRadiansPerSecond, maxAngularSpeedRadiansPerSecondSquared);

        // Basically a PID controller that also uses the physical limits of the swerve (max speed & max acceleration) to not go over the maximum values
        public static final ProfiledPIDController thetaController = 
            new ProfiledPIDController(thetaControllerPID.getKp(), thetaControllerPID.getKi(),thetaControllerPID.getKd(), thetaControllerConstraints);   
        
        // Limits the speeds the trajectory can generate, so it doesn't move over its max speed the robot can handle.    
        public static final TrajectoryConfig trajectoryConfig =
            new TrajectoryConfig(maxSpeedMetersPerSecond, maxAccelerationMetersPerSecondSquared);  
      }

    
}
