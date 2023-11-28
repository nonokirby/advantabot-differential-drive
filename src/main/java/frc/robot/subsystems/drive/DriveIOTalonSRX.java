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

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
//import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

/*import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
/*import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;*/
public class DriveIOTalonSRX implements DriveIO {
  private static final double GEAR_RATIO = 6.0;

  // private final CANSparkMax leftLeader = new CANSparkMax(1, MotorType.kBrushless);
  // private final CANSparkMax rightLeader = new CANSparkMax(2, MotorType.kBrushless);
  // private final CANSparkMax leftFollower = new CANSparkMax(3, MotorType.kBrushless);
  // private final CANSparkMax rightFollower = new CANSparkMax(4, MotorType.kBrushless);
  // private final RelativeEncoder leftEncoder = leftLeader.getEncoder();
  // private final RelativeEncoder rightEncoder = rightLeader.getEncoder();
  private final TalonSRX LLead = new TalonSRX(1);
  private final TalonSRX LFollow = new TalonSRX(2);
  private final TalonSRX RLead = new TalonSRX(3);
  private final TalonSRX RFollow = new TalonSRX(4);

  // private final double LEncoder = LLead.getSelectedSensorPosition();
  // private final double REncoder = RLead.getSelectedSensorPosition();

  // private final Pigeon2 pigeon = new Pigeon2(20);
  // private final StatusSignal<Double> yaw = pigeon.getYaw();
  //private AHRS navx;
  //double angle = navx.getAngle();

  public DriveIOTalonSRX() {
    // leftLeader.restoreFactoryDefaults();
    // rightLeader.restoreFactoryDefaults();
    // leftFollower.restoreFactoryDefaults();
    // rightFollower.restoreFactoryDefaults();
    LLead.configFactoryDefault();
    LLead.setNeutralMode(NeutralMode.Brake);
    LLead.setInverted(false);
    LLead.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

    RLead.configFactoryDefault();
    RLead.setNeutralMode(NeutralMode.Brake);
    RLead.setInverted(true);

    LFollow.configFactoryDefault();
    LFollow.setNeutralMode(NeutralMode.Brake);
    LFollow.follow(LLead);
    LFollow.setInverted(true);

    RFollow.configFactoryDefault();
    RFollow.setNeutralMode(NeutralMode.Brake);
    RFollow.follow(RLead);
    RFollow.setInverted(true);

    // leftLeader.setCANTimeout(250);
    // rightLeader.setCANTimeout(250);
    // leftFollower.setCANTimeout(250);
    // rightFollower.setCANTimeout(250);

    // leftLeader.setInverted(false);
    // rightLeader.setInverted(true);
    // leftFollower.follow(leftLeader, false);
    // rightFollower.follow(rightLeader, false);

    // leftLeader.enableVoltageCompensation(12.0);
    // rightLeader.enableVoltageCompensation(12.0);
    // leftLeader.setSmartCurrentLimit(30);
    // rightLeader.setSmartCurrentLimit(30);

    // leftLeader.burnFlash();
    // rightLeader.burnFlash();
    // leftFollower.burnFlash();
    // rightFollower.burnFlash();

    // pigeon.getConfigurator().apply(new Pigeon2Configuration());
    // pigeon.getConfigurator().setYaw(0.0);

    // yaw.setUpdateFrequency(100.0);

    // pigeon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(DriveIOInputs inputs) {
    inputs.leftPositionRad =
        Units.rotationsToRadians(LLead.getSelectedSensorPosition() / GEAR_RATIO);
    inputs.leftVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(LLead.getSelectedSensorVelocity() / GEAR_RATIO);
    inputs.leftAppliedVolts = LLead.getMotorOutputVoltage() * LLead.getBusVoltage();
    inputs.leftCurrentAmps = new double[] {LLead.getStatorCurrent(), LFollow.getStatorCurrent()};

    inputs.rightPositionRad =
        Units.rotationsToRadians(RLead.getSelectedSensorPosition() / GEAR_RATIO);
    inputs.rightVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(RLead.getSelectedSensorVelocity() / GEAR_RATIO);
    inputs.leftAppliedVolts = RLead.getMotorOutputVoltage() * RLead.getBusVoltage();
    inputs.leftCurrentAmps = new double[] {RLead.getStatorCurrent(), RFollow.getStatorCurrent()};

    // inputs.gyroYaw = Rotation2d.fromDegrees(-yaw.refresh().getValueAsDouble());
    // inputs.gyroYaw = Rotation2d.fromDegrees(navx.getYaw());
  }

  @Override
  public void setOutput(double leftOutput, double rightOutput) {
    LLead.set(TalonSRXControlMode.PercentOutput, leftOutput);
    RLead.set(TalonSRXControlMode.PercentOutput, rightOutput);
  }
}
