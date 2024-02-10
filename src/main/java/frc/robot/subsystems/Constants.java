package frc.robot.subsystems;

import java.util.List;

import org.xml.sax.DocumentHandler;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;

public class Constants {
    
    public static class DriveConstants {
    // The units for the translation2d are in meters and positive x values represent 
    // distance from the center to the front of the bot. Positive y values represent  
    // the distance from the center to the left of the bot. Now I won't forgor :3
    public static final Translation2d frontLeftLocation = new Translation2d(0.8, 0.6985);
    public static final Translation2d frontRightLocation = new Translation2d(0.8, -0.6985);
    public static final Translation2d backLeftLocation = new Translation2d(-0.8, 0.6985);
    public static final Translation2d backRightLocation = new Translation2d(-0.8, -0.6985);

    public static final double wheelDiameter = Units.inchesToMeters(4);

    public static final double driveGearRatio = 6.75;
    public static final double turnGearRatio = 6.75;

     public static final SwerveDriveKinematics driveKinematics =
        new SwerveDriveKinematics(
            frontLeftLocation,
            frontRightLocation,
            backLeftLocation,
            backRightLocation
        );

    public static final double maxSpeedMetresPerSecond = 6;
    public static final double maxAccelerationMetersPerSecond = 5;

    public static final double maxTurningModuleVelocity = 2 * Math.PI;
    public static final double maxTurningModuleAcceleration = 2 * Math.PI;

    public static final double drivePosConversionFactor = (Math.PI * wheelDiameter) / driveGearRatio;
    public static final double turnPosConversionFactor = (2 * Math.PI) / turnGearRatio;

    public static final double driveVelocityConversionFactor = drivePosConversionFactor / 60;
    public static final double turnVelocityConversionFactor = turnPosConversionFactor / 60;

    public static TrajectoryConfig config = new TrajectoryConfig(
      DriveConstants.maxSpeedMetresPerSecond, 
      DriveConstants.maxAccelerationMetersPerSecond
    ).setKinematics(DriveConstants.driveKinematics);

    }
}