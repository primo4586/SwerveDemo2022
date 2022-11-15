package frc.lib.util;

public class SwerveModuleConstants {
    public final int driveMotorID;
    public final int angleMotorID;
    public final int cancoderID;
    public final double angleOffset;
    public final boolean driveInvert;

    /**
     * Swerve Module Constants to be used when creating swerve modules.
     * @param driveMotorID
     * @param angleMotorID
     * @param canCoderID
     * @param angleOffset
     */
    public SwerveModuleConstants(int driveMotorID, int angleMotorID, int canCoderID, double angleOffset, boolean driveInvert) {
        this.driveMotorID = driveMotorID;
        this.angleMotorID = angleMotorID;
        this.driveInvert = driveInvert;
        this.cancoderID = canCoderID;
        this.angleOffset = angleOffset;
    }
}
