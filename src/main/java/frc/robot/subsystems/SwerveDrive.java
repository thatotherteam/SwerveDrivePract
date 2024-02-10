// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.concurrent.CancellationException;


import com.kauailabs.navx.frc.AHRS;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.fasterxml.jackson.databind.ser.std.CalendarSerializer;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.subsystems.Constants.DriveConstants;
import frc.robot.subsystems.SwerveModule;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Angle;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.kinematics.SwerveModulePosition;


public class SwerveDrive extends SubsystemBase {

  private static final double xVal = 0.7493;
  private static final double yVal = 0.7493;
  private static final double maxSpeed = 6;

private SwerveModule frontLeft; 
private SwerveModule frontRight;
private SwerveModule backLeft;
private SwerveModule backRight;

private Odometry odometry;



  public final static SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
     new Translation2d(xVal, yVal),
      new Translation2d(xVal, -yVal),
      new Translation2d(-xVal, yVal),
      new Translation2d(-xVal, -yVal));

      private 

      Field2d field = new Field2d();
      AHRS gyro = new AHRS();

    public SwerveDrive(SwerveModule frontLeft, SwerveModule frontRight, SwerveModule backLeft, SwerveModule backRight) {
      
 frontLeft = new SwerveModule(1, 1, 1, false, false);
 frontRight = new SwerveModule(1, 1, 1, false, false);
backLeft = new SwerveModule(1, 1, 1, true, true);
 backRight = new SwerveModule(1, 1, 1, true, true);

 

 odometry = new SwerveDriveOdometry(
  DRIVE_KINEMATICS, 
  getRotation2d(),
  new SwerveModulePosition[] {
          frontLeft.getPosition(),
          frontRight.getPosition(),
          backLeft.getPosition(),
          backRight.getPosition()
  });
  
}

     private final void getSwervePosition() {
      frontLeft.getPosition();
      frontRight.getPosition();
      backLeft.getPosition();
      backRight.getPosition();

    }

         private final void Velocity() {
      frontLeft.getDriveVelocity();
      frontRight.getDriveVelocity();
      backLeft.getDriveVelocity();
      backRight.getDriveVelocity();
    }

    private final void stop() {
      frontLeft.stop();
      frontRight.stop();
      backLeft.stop();
      backRight.stop();

    }
     public double getGyro() {
        // wraps the raw angle into a -180 to 180 heading
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    public double getGyroRadians() {
        return Math.toRadians(getGyro());
    }

    public Rotation2d getRotation2d() {
        // gets a rotation2D translation of gyro angle
        return Rotation2d.fromDegrees(-getGyro());
    }
   
    public void getState() {
      frontLeft.getState();
      frontRight.getState();
      backLeft.getState();
      backRight.getState();

    }

    public void setMouleStates(SwerveModuleState moduleState) {
     
      
          frontLeft.setModuleState(moduleState);
          frontRight.setModuleState(moduleState);
          backLeft.setModuleState(moduleState);
          backRight.setModuleState(moduleState);
      }
}
