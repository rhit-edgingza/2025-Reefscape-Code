package frc.robot.subsystems.drive;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.SwerveConstants;

public class RealModuleIO implements ModuleIO {
    private TalonFX _driveMotor;
    private TalonFX _rotateMotor;
    private CANcoder _cancoder;
    private Rotation2d _encoderOffset;

    public RealModuleIO(int driveMotorId, int rotateMotorId, Rotation2d encoderOffset) {
        _driveMotor = new TalonFX(driveMotorId, "rio");
        _rotateMotor = new TalonFX(rotateMotorId, "rio");
        _encoderOffset = encoderOffset;
    }

    /** Updates the set of loggable inputs. */
    public void updateInputs(ModuleIOInputs inputs) {
        inputs._drivePositionRad = Units.rotationsToRadians(_driveMotor.getPosition().getValueAsDouble()) / SwerveConstants.driveGearRatio;
        inputs._driveVelocityRadPerSec = Units.rotationsToRadians(_driveMotor.getVelocity().getValueAsDouble()) / SwerveConstants.driveGearRatio; 
        inputs._driveAppliedVolts = _driveMotor.getMotorVoltage().getValueAsDouble();
        inputs._driveCurrentAmps = _driveMotor.getStatorCurrent().getValueAsDouble();

        inputs._turnPosition = Rotation2d.fromRotations(_cancoder.getAbsolutePosition().getValueAsDouble()).minus(_encoderOffset);
        inputs._turnVelocityRadPerSec = Units.rotationsToRadians(_rotateMotor.getVelocity().getValueAsDouble()) / SwerveConstants.angleGearRatio;
        inputs._turnAppliedVolts = _rotateMotor.getMotorVoltage().getValueAsDouble();
        inputs._turnCurrentAmps = _rotateMotor.getStatorCurrent().getValueAsDouble();
    }

    /** Run the drive motor at the specified voltage. */
    public void setDriveVoltage(double volts) {
        _driveMotor.setVoltage(volts);
    }

    /** Run the turn motor at the specified voltage. */
    public void setTurnVoltage(double volts) {
        _rotateMotor.setVoltage(volts);
    }

    /** Enable or disable brake mode on the drive motor. */
    public void setDriveConfig(CurrentLimitsConfigs config) {
        _driveMotor.getConfigurator().apply(config);
    }

    public void setRotateConfig(CurrentLimitsConfigs config) {
        _rotateMotor.getConfigurator().apply(config);
    }

    public void setDriveInverted(boolean inverted) {
        _driveMotor.setInverted(inverted);
    }

    public void setRotationInverted(boolean inverted) {
        _rotateMotor.setInverted(inverted);
    }

    public void resetDriveEncoder() {
        _driveMotor.setPosition(0);
    }
}
