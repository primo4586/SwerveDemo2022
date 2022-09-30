package frc.robot;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

import frc.lib.util.PIDConfig;
import frc.robot.Constants.SwerveConstants;

public final class CTREConfigs {
    public TalonFXConfiguration swerveAngleFXConfig;
    public TalonFXConfiguration swerveDriveFXConfig;
    public CANCoderConfiguration swerveCanCoderConfig;


    // Builds & holds all static configurations for the module motors & cancoders
    public CTREConfigs(){
        swerveAngleFXConfig = new TalonFXConfiguration();
        swerveDriveFXConfig = new TalonFXConfiguration();
        swerveCanCoderConfig = new CANCoderConfiguration();

        /* Swerve Angle Motor Configurations */
        SupplyCurrentLimitConfiguration angleSupplyLimit = new SupplyCurrentLimitConfiguration(
            SwerveConstants.ANGLE_ENABLE_CURRENT_LIMIT, 
            SwerveConstants.ANGLE_CONTINOUS_CURRENT, 
            SwerveConstants.ANGLE_PEAK_CURRENT, 
            SwerveConstants.ANGLE_PEAK_CURRENT_DURATION);

        PIDConfig anglePID = SwerveConstants.ANGLE_PID;    
        swerveAngleFXConfig.slot0.kP = anglePID.getKp();
        swerveAngleFXConfig.slot0.kI = anglePID.getKi();
        swerveAngleFXConfig.slot0.kD = anglePID.getKd();
        swerveAngleFXConfig.slot0.kF = anglePID.getKf();
        swerveAngleFXConfig.supplyCurrLimit = angleSupplyLimit;
        swerveAngleFXConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;


        /* Swerve Drive Motor Configuration */
        SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
            SwerveConstants.DRIVE_ENABLE_CURRENT_LIMIT, 
            SwerveConstants.DRIVE_CONTINOUS_CURRENT, 
            SwerveConstants.DRIVE_PEAK_CURRENT, 
            SwerveConstants.DRIVE_PEAK_CURRENT_DURATION);

        PIDConfig drivePID = SwerveConstants.DRIVE_PID;    
        swerveDriveFXConfig.slot0.kP = drivePID.getKp();
        swerveDriveFXConfig.slot0.kI = drivePID.getKi();
        swerveDriveFXConfig.slot0.kD = drivePID.getKd();
        swerveDriveFXConfig.slot0.kF = drivePID.getKf();        
        swerveDriveFXConfig.supplyCurrLimit = driveSupplyLimit;
        swerveDriveFXConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
        swerveDriveFXConfig.openloopRamp = SwerveConstants.OPEN_LOOP_RAMP;
        swerveDriveFXConfig.closedloopRamp = SwerveConstants.CLOSED_LOOP_RAMP;

        
        /* Swerve CANCoder Configuration */
        swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        swerveCanCoderConfig.sensorDirection = SwerveConstants.CANCODER_INVERT;
        swerveCanCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;

        

    }

}