// Copyright 2021-2023 FRC 6328
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

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;

public class DriveIOTalonFX implements DriveIO {
  private static final double GEAR_RATIO = 6.0;

  private final TalonSRX leftLeader = new TalonSRX(0);
  private final TalonSRX leftFollower = new TalonSRX(1);
  private final TalonSRX rightLeader = new TalonSRX(2);
  private final TalonSRX rightFollower = new TalonSRX(3);

  private final Double leftAppliedVolts = leftLeader.getMotorOutputVoltage();
  private final Double leftLeaderCurrent = leftLeader.getStatorCurrent();
  private final Double leftFollowerCurrent = leftFollower.getStatorCurrent();

  private final Double rightAppliedVolts = rightLeader.getMotorOutputVoltage();
  private final Double rightLeaderCurrent = rightLeader.getStatorCurrent();
  private final Double rightFollowerCurrent = rightFollower.getStatorCurrent();

  private final Pigeon2 pigeon = new Pigeon2(20);
  private final StatusSignal<Double> yaw = pigeon.getYaw();

  public DriveIOTalonFX() {
    var config = new TalonSRXConfiguration();
    config.continuousCurrentLimit = 30;
    leftFollower.getConfigurator().apply(config);
    rightLeader.getConfigurator().apply(config);
    rightFollower.getConfigurator().apply(config);
    leftFollower.setControl(new Follower(leftLeader.getDeviceID(), false));
    rightFollower.follow(rightLeader.getDeviceID(),rightFollower, false);
  }
  @Override
  public void updateInputs(DriveIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        leftPosition,
        leftVelocity,
        leftAppliedVolts,
        leftLeaderCurrent,
        leftFollowerCurrent,
        rightPosition,
        rightVelocity,
        rightAppliedVolts,
        rightLeaderCurrent,
        rightFollowerCurrent,
        yaw);

    inputs.leftPositionRad = Units.rotationsToRadians(leftPosition.getValueAsDouble()) / GEAR_RATIO;
    inputs.leftVelocityRadPerSec =
        Units.rotationsToRadians(leftVelocity.getValueAsDouble()) / GEAR_RATIO;
    inputs.leftAppliedVolts = leftAppliedVolts.getValueAsDouble();
    inputs.leftCurrentAmps =
        new double[] {leftLeaderCurrent.getValueAsDouble(), leftFollowerCurrent.getValueAsDouble()};

    inputs.rightPositionRad =
        Units.rotationsToRadians(rightPosition.getValueAsDouble()) / GEAR_RATIO;
    inputs.rightVelocityRadPerSec =
        Units.rotationsToRadians(rightVelocity.getValueAsDouble()) / GEAR_RATIO;
    inputs.rightAppliedVolts = rightAppliedVolts.getValueAsDouble();
    inputs.rightCurrentAmps =
        new double[] {
          rightLeaderCurrent.getValueAsDouble(), rightFollowerCurrent.getValueAsDouble()
        };

    inputs.gyroYaw = Rotation2d.fromDegrees(-yaw.getValueAsDouble());
  }

  @Override
  public void setVoltage(double leftVolts, double rightVolts) {
    leftLeader.setControl(new VoltageOut(leftVolts));
    rightLeader.setControl(new VoltageOut(rightVolts));
  }
}
