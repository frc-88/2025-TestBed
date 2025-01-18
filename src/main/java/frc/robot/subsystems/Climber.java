// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;
import frc.robot.util.preferenceconstants.PIDPreferenceConstants;

public class Climber extends SubsystemBase {
  private DoublePreferenceConstant p_maxVelocity = new DoublePreferenceConstant("Climber/MotionMagicVelocity", 100);
  private DoublePreferenceConstant p_maxAcceleration = new DoublePreferenceConstant("Climber/MotionMagicAcceleration",
      1000);
  private DoublePreferenceConstant p_maxJerk = new DoublePreferenceConstant("Climber/MotionMagicJerk", 100000);
  private PIDPreferenceConstants p_PidPreferenceConstants = new PIDPreferenceConstants("Climber/PID", 0.0, 0.0, 0.0,
      0.12, 0.0, 0.0, 0.0, 0.0);
  private DoublePreferenceConstant p_pivotLimit = new DoublePreferenceConstant("Climber/PivotLimit", 100);
  private DoublePreferenceConstant p_gripperLimit = new DoublePreferenceConstant("Climber/GripperLimit", 100);

  private final TalonFX m_pivot = new TalonFX(Constants.CLIMBER_PIVOT_MOTOR, Constants.RIO_CANBUS);
  private final TalonFX m_gripper = new TalonFX(Constants.CLIMBER_GRIPPER_MOTOR, Constants.RIO_CANBUS);

  private final DutyCycleOut m_pivotRequest = new DutyCycleOut(0.0);
  private final DutyCycleOut m_gripperRequest = new DutyCycleOut(0.0);

  /** Creates a new Climber. */
  public Climber() {
    configureTalons();
    m_gripper.setPosition(0.0);
    m_pivot.setPosition(0.0);
  }

  private void configureTalons() {
    TalonFXConfiguration cfg = new TalonFXConfiguration();

    MotionMagicConfigs mm = cfg.MotionMagic;
    mm.MotionMagicCruiseVelocity = p_maxVelocity.getValue();
    mm.MotionMagicAcceleration = p_maxAcceleration.getValue();
    mm.MotionMagicJerk = p_maxJerk.getValue();

    Slot0Configs slot0 = cfg.Slot0;
    slot0.kP = p_PidPreferenceConstants.getKP().getValue();
    slot0.kI = p_PidPreferenceConstants.getKI().getValue();
    slot0.kD = p_PidPreferenceConstants.getKD().getValue();
    slot0.kV = p_PidPreferenceConstants.getKF().getValue();
    slot0.kS = p_PidPreferenceConstants.getKS().getValue(); // Approximately 0.25V to get the mechanism moving

    SoftwareLimitSwitchConfigs softLimits = cfg.SoftwareLimitSwitch;
    softLimits.ForwardSoftLimitEnable = true;

    softLimits.ForwardSoftLimitThreshold = p_pivotLimit.getValue();
    m_pivot.getConfigurator().apply(cfg);

    softLimits.ForwardSoftLimitThreshold = p_gripperLimit.getValue();
    m_gripper.getConfigurator().apply(cfg);
  }

  public void setPivot(double speed) {
    m_pivot.setControl(m_pivotRequest.withOutput(speed));
  }

  public void setGripper(double speed) {
    m_gripper.setControl(m_gripperRequest.withOutput(speed));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
