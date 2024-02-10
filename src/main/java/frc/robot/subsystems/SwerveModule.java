package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Constants.DriveConstants;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {

    // #region -*-*- Constants -*-*-

    private final double WHEEL_DIAMETER = Units.inchesToMeters(4);
    private final double WHEEL_CIRC = WHEEL_DIAMETER * Math.PI;

    private final double DRIVE_GEAR_RATIO = 6.75;

    private final double drivePosConversionFactor = WHEEL_CIRC / DRIVE_GEAR_RATIO;
    private final double driveVelocityConversionFactor = drivePosConversionFactor / 60;

    // probably **INCORRECT BUZZER**
    private final double maxMetersPerSec = 6;

    private final double directionPID_P = 0.5;
    private final double directionPID_I = 0;
    private final double directionPID_D = 0;
    // #endregion

    private final CANSparkMax driveMotor;
    private final RelativeEncoder driveEncoder;
    private final CANSparkMax directionMotor;
    private final CANcoder directionEncoder;
    private final PIDController directionPID;
    private SwerveModuleState optState;
    private double driveMotorPercent;
    private double directionMotorPercent;
    private double pie = Math.PI * 2;

    public SwerveModule(Integer driveMotorID, Integer directionMotorID, Integer CANCoderID,
            boolean isDriveMotorReversed, boolean isDirectionEncoderReversed) {

        driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        directionMotor = new CANSparkMax(directionMotorID, MotorType.kBrushless);

        driveMotor.setInverted(isDriveMotorReversed);
        directionMotor.setInverted(isDirectionEncoderReversed);

        driveEncoder = driveMotor.getEncoder();
        directionEncoder = new CANcoder(CANCoderID);

        driveEncoder.setPositionConversionFactor(DriveConstants.drivePosConversionFactor);
        driveEncoder.setVelocityConversionFactor(DriveConstants.driveVelocityConversionFactor);

        directionPID = new PIDController(0.5, 0, 0);
        directionPID.enableContinuousInput(-Math.PI, Math.PI);

        driveMotor.setIdleMode(IdleMode.kBrake);
        directionMotor.setIdleMode(IdleMode.kBrake);
    }

    public double getDirectionPos() {
        // + 0.5 to get the position to 0-1 and * 360 to make number degrees
        return (directionEncoder.getAbsolutePosition().getValueAsDouble() + 0.5) * pie;
    }

    double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    private double getDrivePos() {
        return driveEncoder.getPosition();
    }

    public void stop() {
        driveMotor.set(0);
        directionMotor.set(0);
    }

    void setBreak() {
        driveMotor.setIdleMode(IdleMode.kBrake);
        directionMotor.setIdleMode(IdleMode.kBrake);
    }

    private void setCoast() {
        driveMotor.setIdleMode(IdleMode.kBrake);
        directionMotor.setIdleMode(IdleMode.kBrake);

    }

    public void resetEncod() {
        driveEncoder.setPosition(0);
        directionEncoder.setPosition(0);

    }
public SwerveModulePosition getPosition(){
 return new SwerveModulePosition(getDrivePos(), new Rotation2d(getDirectionPos()));
}
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getDirectionPos()));
    }

    public void setModuleState(SwerveModuleState moduleState) {
        directionPID.setSetpoint(moduleState.angle.getRadians());
        optState = SwerveModuleState.optimize(optState, getState().angle);
        driveMotorPercent = optState.speedMetersPerSecond / maxMetersPerSec;
        directionMotorPercent = directionPID.calculate(getDirectionPos());

        driveMotor.set(driveMotorPercent);
        directionMotor.set(directionMotorPercent);

    
  
}
}
