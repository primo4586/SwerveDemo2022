package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.SwerveModule;
import frc.robot.Constants.SwerveConstants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;

import javax.swing.plaf.TreeUI;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.TeleopSwerve;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public PigeonIMU gyro;

    public Swerve() {
        gyro = new PigeonIMU(new TalonSRX(SwerveConstants.PIGEON_ID));
        gyro.configFactoryDefault();
        zeroGyro();
        
        swerveOdometry = new SwerveDriveOdometry(SwerveConstants.KINEMATICS, getYaw());

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, SwerveConstants.FrontLeftModule.CONSTANTS),
            new SwerveModule(1, SwerveConstants.FrontRightModule.CONSTANTS),
            new SwerveModule(2, SwerveConstants.BackLeftModule.CONSTANTS),
            new SwerveModule(3, SwerveConstants.BackRightModule.CONSTANTS)
        };
      
    }

    /**
     * Drives the robot during teleop control.
     * @see TeleopSwerve
     * 
     * @param translation Movement of the robot on the X & Y plane
     * @param rotation Movement in rotation. 
     * @param fieldRelative If the robot should move relative to the field or the robot.
     * @param isOpenLoop If the robot should use PID & FF to correct itself and be more accurate 
     */
    public void teleopDrive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.SwerveConstants.KINEMATICS.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getYaw()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.MAX_SPEED);

        //mSwerveMods[1].setDesiredState(swerveModuleStates[1], isOpenLoop);
         for(SwerveModule mod : mSwerveMods){
             mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
         }
    }    

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.MAX_SPEED); // Checks the modules don't exceed their max speed.
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }    

    public void setModuleStateRotation(int moduleNumber, double rotation) {
        mSwerveMods[moduleNumber].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(rotation)), true);
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(pose, getYaw());
    }

    public SwerveModuleState[] getStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public void zeroGyro(){
        gyro.setYaw(0);
    }

    public Rotation2d getYaw() {
        double[] ypr = new double[3];
        gyro.getYawPitchRoll(ypr);
        return (SwerveConstants.INVERT_GYRO) ? Rotation2d.fromDegrees(360 - ypr[0]) : Rotation2d.fromDegrees(ypr[0]);
    }

    @Override
    public void periodic(){
        swerveOdometry.update(getYaw(), getStates());  

        // TODO: When we're done calibrating all the constants, we can just remove this to save a bit of network, not super important though.
        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + "Vel",mod.getState().speedMetersPerSecond);
        }
        SmartDashboard.putNumber("Gyro", getYaw().getDegrees());
    }

    public void stopModules() {
        for(SwerveModule module : mSwerveMods) {
            module.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)), true);
        }
    }

    public void setStateForModule(int moduleNum, double x, double y, double rotation) {
    }
}