package frc.robot.subsystems;

import java.sql.Time;
import java.util.List;
import java.util.function.Supplier;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import frc.robot.SwerveModule;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants;
import frc.robot.PoseEstimate;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    private SwerveDriveOdometry swerveOdometry;
    private SwerveModule[] mSwerveMods;
    private PigeonIMU gyro;
    private PoseEstimate poseEstimateClass;// TODO: put a better name

    private Field2d field2d = new Field2d();
    private double teleopRotationOffset = 0;

    // odometry estimator based on camera
    private final SwerveDrivePoseEstimator poseEstimation;

    public Swerve() {
        poseEstimateClass = new PoseEstimate();
        gyro = new PigeonIMU(new TalonSRX(Constants.SwerveConstants.pigeonID));
        gyro.configFactoryDefault();
        zeroGyro();

        SmartDashboard.putData(field2d);
        mSwerveMods = new SwerveModule[] {
                new SwerveModule(0, Constants.SwerveConstants.Mod0.constants),
                new SwerveModule(1, Constants.SwerveConstants.Mod1.constants),
                new SwerveModule(2, Constants.SwerveConstants.Mod2.constants),
                new SwerveModule(3, Constants.SwerveConstants.Mod3.constants)
        };

        swerveOdometry = new SwerveDriveOdometry(Constants.SwerveConstants.swerveKinematics, getYaw(), getPositions());
        poseEstimation = new SwerveDrivePoseEstimator(Constants.SwerveConstants.swerveKinematics, getYaw(),
                getPositions(), swerveOdometry.getPoseMeters());
        poseEstimation.setVisionMeasurementStdDevs(VecBuilder.fill(0.3, 0.3, 0.3));
    }

    /**
     * Drives the robot during teleop control.
     * 
     * @see TeleopSwerve
     * 
     * @param translation   Movement of the robot on the X & Y plane
     * @param rotation      Movement in rotation.
     * @param fieldRelative If the robot should move relative to the field or the
     *                      robot.
     * @param isOpenLoop    If the robot should use PID & FF to correct itself and
     *                      be more accurate
     */
    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        // use less ternary operator and state more explicit local varibles in the
        // function
        SwerveModuleState[] swerveModuleStates = Constants.SwerveConstants.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation,
                        isOpenLoop ? getTeleopYaw() : getYaw())
                        : new ChassisSpeeds(
                                translation.getX(),
                                translation.getY(),
                                rotation));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.SwerveConstants.maxSpeed);

        // mSwerveMods[1].setDesiredState(swerveModuleStates[1], isOpenLoop);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    public void stopModules() {
        for (SwerveModule module : mSwerveMods) {
            module.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)), true);
        }
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.SwerveConstants.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public Pose2d getPose() {
        return poseEstimation.getEstimatedPosition();
    }

    public void setFieldTrajectory(String name, Trajectory trajectory) {
        field2d.getObject(name).setTrajectory(trajectory);
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getPositions(), pose);
    }

    public SwerveModuleState[] getStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : mSwerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public void zeroGyro() {
        gyro.setYaw(0);
    }

    public void zeroTeleopGyro() {
        teleopRotationOffset = getYaw().getDegrees();
    }

    public Rotation2d getYaw() {
        double[] ypr = new double[3];
        gyro.getYawPitchRoll(ypr);
        return (Constants.SwerveConstants.invertGyro) ? Rotation2d.fromDegrees(360 - ypr[0])
                : Rotation2d.fromDegrees(ypr[0]);
    }

    public Rotation2d getTeleopYaw() {
        return getYaw().minus(Rotation2d.fromDegrees(teleopRotationOffset));
    }

    @Override
    public void periodic() {
        swerveOdometry.update(getYaw(), getPositions());

        SmartDashboard.putNumber("Gyro", getYaw().getDegrees());
        SmartDashboard.putNumber("Teleop Gyro", getTeleopYaw().getDegrees());
        updateOdometry();

        for (SwerveModule mod : mSwerveMods) {
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
        }
    }

    public void updateOdometry() {
        poseEstimation.update(getYaw(), getPositions());
        field2d.getObject("Odometry").setPose(swerveOdometry.getPoseMeters());

        // Also apply vision measurements. We use 0.3 seconds in the past as an example
        // -- on
        // a real robot, this must be calculated based either on latency or timestamps.
        Pair<Pose3d, Double> result = poseEstimateClass.getEstimatedGlobalPose(poseEstimation.getEstimatedPosition());
        var camPose = result.getFirst();
        var camPoseObsTime = result.getSecond();
        if (camPose != null) {
            poseEstimation.addVisionMeasurement(camPose.toPose2d(), camPoseObsTime);
            field2d.getObject("Vision position").setPose(camPose.toPose2d());
        }
        field2d.setRobotPose(getPose());
    }

    public void resetToAbsoluteModules() {
        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(mod.getOffset())), true);
        }
    }

    public SwerveModulePosition[] getPositions() {
        return new SwerveModulePosition[] {
                mSwerveMods[0].getPostion(),
                mSwerveMods[1].getPostion(),
                mSwerveMods[2].getPostion(),
                mSwerveMods[3].getPostion(),
        };
    }

    public Command trajectoryToTag(int tagID) {
        Supplier<Command> followCmdSupplier = () -> new PPSwerveControllerCommand(generateTrajectory(tagID),
                this::getPose,
                SwerveConstants.swerveKinematics,
                new PIDController(AutoConstants.kPXController, 0, 0),
                new PIDController(AutoConstants.kPYController, 0, 0),
                new PIDController(AutoConstants.kPThetaController, 0, 0),
                this::setModuleStates,
                false,
                this);

        return new ProxyCommand(followCmdSupplier);
    }

    public PathPlannerTrajectory generateTrajectory(int tagID) {
        PathPoint robotPose = new PathPoint(getPose().getTranslation(), getYaw());
        Pose2d targetPosition = poseEstimateClass.getApriltagLayout().getTagPose(tagID).get().toPose2d();
        var goalOffsetFromTarget = new Translation2d(1, 0.3);
        
        // better name, not easy to understand
        var goalTranslation = targetPosition.getTranslation().minus(goalOffsetFromTarget);
        PathPoint endPoint = new PathPoint(goalTranslation, Rotation2d.fromDegrees(0));

        return PathPlanner.generatePath(
                new PathConstraints(AutoConstants.kMaxSpeedMetersPerSecond,
                        AutoConstants.kMaxAccelerationMetersPerSecondSquared),
                List.of(robotPose, endPoint));
    }
}