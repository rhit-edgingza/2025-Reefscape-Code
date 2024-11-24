// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;

public interface GyroIO {
  @AutoLog
  public static class GyroIOInputs {
    public double _pigeonSensorYaw = 0.0;
    public double _pigeonSensorPitch = 0.0;
    public double _pigeonSensorRoll = 0.0;
    public double _pigeonCompassHeading = 0.0;

  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(GyroIOInputs inputs) {}

    public default void zeroPigeon() {}

    public default void setHeadingForward() {}
}
