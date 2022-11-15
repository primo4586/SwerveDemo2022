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
        public static final int PIGEON_ID = 1;
        public static final boolean INVERT_GYRO = false; // Always ensure Gyro is CCW+ CW-

        /* Drivetrain Constants (Physical Constants) */
        /** Distance between the left and right modules wheels (in meters) */
        public static final double TRACKWIDTH = 0.61;
        /** Distance between the front and rear module wheels (in meters) */
        public static final double WHEEL_BASE = 0.61;
        /** "קוטר" */
        public static final double WHEEL_DIAMETER = Units.inchesToMeters(3.9); 
        /** "היקף" */
        public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

        /* Ramping - how fast the motors are/can move from 0 speed to full speed */
        public static final double OPEN_LOOP_RAMP = 0.25;
        public static final double CLOSED_LOOP_RAMP = 0.0;

        /* Gear Ratios */
        public static final double DRIVE_GEAR_RATIO = (8.14 / 1.0); //8.14:1
        public static final double ANGLE_GEAR_RATIO = (150 / 7) / 1.0; // (150 / 7) : 1

        /** 
         * Swerve Kinematics, essentialy the physical "outline" of the robot and where modules are placed relative to the robot center 
         * while, Positive X is towards the direction of the robot (front of the robot) and Positive Y is to the left side of the robot.
         * @see {https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/coordinate-systems.html}
         * (Order is Front Left, Front Right, Back Left, Back Right)
         */
        public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
                new Translation2d(WHEEL_BASE / 2.0, TRACKWIDTH / 2.0),
                new Translation2d(WHEEL_BASE / 2.0, -TRACKWIDTH / 2.0),
                new Translation2d(-WHEEL_BASE / 2.0, TRACKWIDTH / 2.0),
                new Translation2d(-WHEEL_BASE / 2.0, -TRACKWIDTH / 2.0));

        /* Swerve Current Limiting */
        public static final int ANGLE_CONTINOUS_CURRENT = 25;
        public static final int ANGLE_PEAK_CURRENT = 40;
        public static final double ANGLE_PEAK_CURRENT_DURATION = 0.1;
        public static final boolean ANGLE_ENABLE_CURRENT_LIMIT = true;

        public static final int DRIVE_CONTINOUS_CURRENT = 35;
        public static final int DRIVE_PEAK_CURRENT = 60;
        public static final double DRIVE_PEAK_CURRENT_DURATION = 0.1;
        public static final boolean DRIVE_ENABLE_CURRENT_LIMIT = true;

        /* Angle Motor PID Values */
       public static final PIDConfig ANGLE_PID = new PIDConfig(0, 0, 0,0);

        /* Drive Motor PID Values */
        public static final PIDConfig DRIVE_PID = new PIDConfig(0, 0, 0, 0);

        /* Drive Motor Characterization Values */
        public static final double DRIVE_KS = (0 / 12); //divide by 12 to convert from volts to percent output for CTRE
        public static final double DRIVE_KV = (0 / 12);
        public static final double DRIVE_KA = (0 / 12);
        public static final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(SwerveConstants.DRIVE_KS,SwerveConstants.DRIVE_KV, SwerveConstants.DRIVE_KA);

        /* Swerve Profiling Values */
        public static final double MAX_SPEED = 4.5; //meters per second
        public static final double MAX_ANGULAR_VELOCITY = 11.5;

        /* Neutral Modes */
        public static final NeutralMode ANGLE_NEUTRAL = NeutralMode.Coast;
        public static final NeutralMode DRIVE_NEUTRAL = NeutralMode.Brake;

        /* Motor Inverts */
        public static final boolean DRIVE_MOTOR_INVERT = false;
        public static final boolean ANGLE_MOTOR_INVERT = false;

        /* Angle Encoder Invert */
        public static final boolean CANCODER_INVERT = false;

        /* Module Specific Constants */
        /** Front Left Module - Module 0 */
        public static final class FrontLeftModule {
            public static final int DRIVE_MOTOR_ID = 1;
            public static final int ANGLE_MOTOR_ID = 2;
            public static final int CANCODER_ID = 1;
            public static final double ANGLE_OFFSET = 0;
            public static final SwerveModuleConstants CONSTANTS = 
                new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CANCODER_ID, ANGLE_OFFSET);
        }

        /** Front Right Module - Module 1 */
        public static final class FrontRightModule {
            public static final int DRIVE_MOTOR_ID = 3;
            public static final int ANGLE_MOTOR_ID = 4;
            public static final int CANCODER_ID = 2;
            public static final double ANGLE_OFFSET = 0;
            public static final SwerveModuleConstants CONSTANTS = 
                new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CANCODER_ID, ANGLE_OFFSET);
        }
        
        /** Back Left Module - Module 2 */
        public static final class BackLeftModule {
            public static final int DRIVE_MOTOR_ID = 5;
            public static final int ANGLE_MOTOR_ID = 6;
            public static final int CANCODER_ID = 3;
            public static final double ANGLE_OFFSET = 0;
            public static final SwerveModuleConstants CONSTANTS = 
                new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CANCODER_ID, ANGLE_OFFSET);
        }

        /** Back Right Module - Module 3 */
        public static final class BackRightModule {
            public static final int DRIVE_MOTOR_ID = 7;
            public static final int ANGLE_MOTOR_ID = 8;
            public static final int CANCODER_ID = 4;
            public static final double ANGLE_OFFSET = 0;
            public static final SwerveModuleConstants CONSTANTS = 
                new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CANCODER_ID, ANGLE_OFFSET);
        }

    }

    public static final class AutoConstants {
        // Self-explanatory, Limits for speed (linear velocity) and speed of rotation (angular velocity)
        public static final double MAX_SPEED_METERS = 3;
        public static final double MAX_ACCELERATION_METERS_SQRD = 3;
        public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = Math.PI;
        public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQRD = Math.PI;
    
        /**
         * These are SEPARATE from the Drive/Module PID Controllers, while the drive PID corrects errors from target velocity/setpoint for each motor,
         * These controllers are used to calculate the target velocities based off errors from the current robot pose to the target pose. (in trajectories)
         * Essentially changes what should be the target velocity be given to each motor, giving the pose error from where we are, to the next trajectory pose.
         * {@see FollowTrajectory} {@see HolonomicDriveController}
         */
        public static final PIDConfig X_CONFIG = new PIDConfig(1, 0, 0);
        public static final PIDConfig Y_CONFIG = new PIDConfig(1, 0, 0);
        public static final PIDConfig THETA_CONFIG = new PIDConfig(1, 0, 0);
    
        // Constraint for the motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints THETA_TRAPEZOID_CONSTRAINTS =
            new TrapezoidProfile.Constraints(
                MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQRD);

        // Basically a PID controller that also uses the physical limits of the swerve (max speed & max acceleration) to not go over the maximum values
        public static final ProfiledPIDController THETA_CONTROLLER = 
            new ProfiledPIDController(THETA_CONFIG.getKp(), THETA_CONFIG.getKi(),THETA_CONFIG.getKd(), THETA_TRAPEZOID_CONSTRAINTS);   
        
        // Limits the speeds the trajectory can generate, so it doesn't move over its max speed the robot can handle.    
        public static final TrajectoryConfig TRAJECTORY_CONFIG =
            new TrajectoryConfig(MAX_SPEED_METERS, MAX_ACCELERATION_METERS_SQRD);  
      }

    
}
