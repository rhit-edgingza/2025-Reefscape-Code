package frc.robot.subsystems.drive;

import com.ctre.phoenix.sensors.Pigeon2Configuration;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;

public class RealGyroIO implements GyroIO {
    private TalonFX _driveMotor;
    private TalonFX _rotateMotor;
    private CANcoder _cancoder;
    private Rotation2d _encoderOffset;


    public static WPI_Pigeon2 _pigeonSensor;
    
    private Pigeon2Configuration _pigeonConfig;
    

    public RealGyroIO() {
        _pigeonSensor = new WPI_Pigeon2(Constants.SwerveConstants.PIGEON_SENSOR_ID);
        _pigeonConfig = new Pigeon2Configuration();
        _pigeonSensor.configFactoryDefault();
        _pigeonSensor.reset();
        zeroPigeon();

        _pigeonSensor.getAllConfigs(_pigeonConfig);
    }

    /** Updates the set of loggable inputs. */
    public void updateInputs(GyroIOInputs inputs) {
        inputs._pigeonSensorYaw = _pigeonSensor.getYaw();
        inputs._pigeonSensorPitch = _pigeonSensor.getPitch();
        inputs._pigeonSensorRoll = _pigeonSensor.getRoll();
        inputs._pigeonCompassHeading =_pigeonSensor.getCompassHeading();
       
    }

    public void zeroPigeon() {
        _pigeonSensor.reset();
    }

    public WPI_Pigeon2 getPigeonSensor() {
        return _pigeonSensor;
    }

    public void setHeadingForward() {
    _pigeonSensor.setYaw(0.0);
     }


}
