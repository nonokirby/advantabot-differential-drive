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
import com.ctre.phoenix.motorcontrol.

public class DriveIOTalonFX implements DriveIO {
  private static final double GEAR_RATIO = 6.0;

  private final VictorSPX leftLeader = new VictorSPX(0);
  private final VictorSPX leftFollower = new VictorSPX(1);
  private final VictorSPX rightLeader = new VictorSPX(2);
  private final VictorSPX rightFollower = new VictorSPX(3);

  private final StatusSignal<Double> leftAppliedVolts = leftLeader.getMotorVoltage();
  private final StatusSignal<Double> leftLeaderCurrent = leftLeader.getStatorCurrent();
  private final StatusSignal<Double> leftFollowerCurrent = leftFollower.getStatorCurrent();

  private final StatusSignal<Double> rightAppliedVolts = rightLeader.getMotorVoltage();
  private final StatusSignal<Double> rightLeaderCurrent = rightLeader.getStatorCurrent();
  private final  rightFollowerCurrent = rightFollower.getStatorCurrent();

  private final Pigeon2 pigeon = new Pigeon2(20);
  private final StatusSignal<Double> yaw = pigeon.getYaw();

  public DriveIOTalonFX() {
    var config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = 30.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    leftLeader.getConfigurator().apply(config);
    leftFollower.getConfigurator().apply(config);
    rightLeader.getConfigurator().apply(config);
    rightFollower.getConfigurator().apply(config);
    leftFollower.setControl(new Follower(leftLeader.getDeviceID(), false));
    rightFollower.setControl(new Follower(rightLeader.getDeviceID(), false));

    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0, leftPosition, rightPosition, yaw); // Required for odometry, use faster rate
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        leftVelocity,
        leftAppliedVolts,
        leftLeaderCurrent,
        leftFollowerCurrent,
        rightVelocity,
        rightAppliedVolts,
        rightLeaderCurrent,
        rightFollowerCurrent);
    leftLeader.optimizeBusUtilization();
    leftFollower.optimizeBusUtilization();
    rightLeader.optimizeBusUtilization();
    rightFollower.optimizeBusUtilization();
    pigeon.optimizeBusUtilization();
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
