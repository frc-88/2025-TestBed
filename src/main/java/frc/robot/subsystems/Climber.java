// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;
import frc.robot.util.preferenceconstants.PIDPreferenceConstants;

public class Climber extends SubsystemBase {
  private DoublePreferenceConstant p_grippermaxVelocity = new DoublePreferenceConstant("Climber/gripperMotionMagicVelocity", 100);
  private DoublePreferenceConstant p_grippermaxAcceleration = new DoublePreferenceConstant("Climber/gripperMotionMagicAcceleration",
      1000);
  private DoublePreferenceConstant p_grippermaxJerk = new DoublePreferenceConstant("Climber/gripperMotionMagicJerk", 100000);
  private PIDPreferenceConstants p_gripperPidPreferenceConstants = new PIDPreferenceConstants("Climber/gripperPID", 0.0, 0.0, 0.0,
      0.12, 0.0, 0.0, 0.0, 0.0);

  private DoublePreferenceConstant p_pivotmaxVelocity = new DoublePreferenceConstant("Climber/pivotMotionMagicVelocity", 100);
  private DoublePreferenceConstant p_pivotmaxAcceleration = new DoublePreferenceConstant("Climber/pivotMotionMagicAcceleration",
      1000);
  private DoublePreferenceConstant p_pivotmaxJerk = new DoublePreferenceConstant("Climber/pivotMotionMagicJerk", 100000);
  private PIDPreferenceConstants p_pivotPidPreferenceConstants = new PIDPreferenceConstants("Climber/pivotPID", 0.0, 0.0, 0.0,
      0.12, 0.0, 0.0, 0.0, 0.0);

  private DoublePreferenceConstant p_pivotLimit = new DoublePreferenceConstant("Climber/PivotLimit", 100);
  private DoublePreferenceConstant p_gripperLimit = new DoublePreferenceConstant("Climber/GripperLimit", 100);
  private DoublePreferenceConstant p_gripperStowSpeed = new DoublePreferenceConstant("Climber/GripperStowSpeed",0.01);
  private DoublePreferenceConstant p_pivotStowSpeed = new DoublePreferenceConstant("Climber/PivotStowSpeed", 0.01);

  private DoublePreferenceConstant p_gripperClosedTorque = new DoublePreferenceConstant("Climber/gripperClosedTorque", 0.0);
  private DoublePreferenceConstant p_gripperStallTorque = new DoublePreferenceConstant("Climber/gripperStallTorque", 0.0);
  private DoublePreferenceConstant p_pivotTorque = new DoublePreferenceConstant("Climber/PivotTorque", 0.0);

  private final TalonFX m_pivot = new TalonFX(Constants.CLIMBER_PIVOT_MOTOR, Constants.RIO_CANBUS);
  private final TalonFX m_gripper = new TalonFX(Constants.CLIMBER_GRIPPER_MOTOR, Constants.RIO_CANBUS);

  private boolean isCalibrated = false;
  private double kPivotMotorRotationsToClimberPosition = Constants.PIVOT_MOTOR_ROTATIONS_TO_CLIMBER_POSITION;
  private double kGripperMotorRotationsToPosition = Constants.GRIPPER_MOTOR_ROTATIONS_TO_POSITION;

  private final DutyCycleOut m_pivotRequest = new DutyCycleOut(0.0);
  private final DutyCycleOut m_gripperRequest = new DutyCycleOut(0.0);
  private MotionMagicVoltage m_motionMagic = new MotionMagicVoltage(0.0);
  private TorqueCurrentFOC gripperClosedtorque = new TorqueCurrentFOC(p_gripperClosedTorque.getValue()).withMaxAbsDutyCycle(0.75);
  private TorqueCurrentFOC gripperStalltorque = new TorqueCurrentFOC(p_gripperStallTorque.getValue()).withMaxAbsDutyCycle(0.25);
  private TorqueCurrentFOC pivottorque = new TorqueCurrentFOC(p_pivotTorque.getValue()).withMaxAbsDutyCycle(0.25);
  
  private Debouncer climberDebouncer = new Debouncer(1.0);

  
    /** Creates a new Climber. */
    public Climber() {
      configureTalons();
      m_gripper.setPosition(0.0);
      m_pivot.setPosition(0.0);
    }
  
    private void configureTalons() {
      TalonFXConfiguration pivotcfg = new TalonFXConfiguration();
      TalonFXConfiguration grippercfg = new TalonFXConfiguration();
  
      MotionMagicConfigs pivot_mm = pivotcfg.MotionMagic;
      MotionMagicConfigs gripper_mm = grippercfg.MotionMagic;

      gripper_mm.MotionMagicCruiseVelocity = p_grippermaxVelocity.getValue();
      gripper_mm.MotionMagicAcceleration = p_grippermaxAcceleration.getValue();
      gripper_mm.MotionMagicJerk = p_grippermaxJerk.getValue();

      pivot_mm.MotionMagicCruiseVelocity = p_pivotmaxVelocity.getValue();
      pivot_mm.MotionMagicAcceleration = p_pivotmaxAcceleration.getValue();
      pivot_mm.MotionMagicJerk = p_pivotmaxJerk.getValue();
  
      Slot0Configs pivotslot0 = pivotcfg.Slot0;
      Slot0Configs gripperslot0 = pivotcfg.Slot0;

      gripperslot0.kP = p_gripperPidPreferenceConstants.getKP().getValue();
      gripperslot0.kI = p_gripperPidPreferenceConstants.getKI().getValue();
      gripperslot0.kD = p_gripperPidPreferenceConstants.getKD().getValue();
      gripperslot0.kV = p_gripperPidPreferenceConstants.getKF().getValue();
      gripperslot0.kS = p_gripperPidPreferenceConstants.getKS().getValue();
      
      pivotslot0.kP = p_pivotPidPreferenceConstants.getKP().getValue();
      pivotslot0.kI = p_pivotPidPreferenceConstants.getKI().getValue();
      pivotslot0.kD = p_pivotPidPreferenceConstants.getKD().getValue();
      pivotslot0.kV = p_pivotPidPreferenceConstants.getKF().getValue();
      pivotslot0.kS = p_pivotPidPreferenceConstants.getKS().getValue();// Approximately 0.25V to get the mechanism moving
  
      SoftwareLimitSwitchConfigs softLimits = pivotcfg.SoftwareLimitSwitch;
      softLimits.ReverseSoftLimitEnable = true;
      softLimits.ReverseSoftLimitThreshold = p_pivotLimit.getValue();

      VoltageConfigs grippervoltage = grippercfg.Voltage;
      grippervoltage.PeakForwardVoltage = 9.0;

      SoftwareLimitSwitchConfigs gripperSoftLimits = grippercfg.SoftwareLimitSwitch;
      gripperSoftLimits.ForwardSoftLimitEnable = false;
      gripperSoftLimits.ForwardSoftLimitThreshold = p_gripperLimit.getValue();

      m_pivot.getConfigurator().apply(pivotcfg);
  
      m_gripper.getConfigurator().apply(grippercfg);
    }

    public void pivotNeutralGrabberOpen() {
      m_pivot.setNeutralMode(NeutralModeValue.Coast);
      m_gripper.setControl(gripperStalltorque);
    }

    public void pivotNeutralGrabberClosed() {
      m_pivot.setNeutralMode(NeutralModeValue.Coast);
      m_gripper.setControl(gripperClosedtorque);
    }

    public void pivotUpGrabberClosed() {
      m_pivot.setControl(pivottorque);
      m_gripper.setControl(gripperClosedtorque);
    }
  
    public void stow() {
      if (isCalibrated) {
          m_gripper.setControl(m_motionMagic.withPosition(0.0));
          m_pivot.setControl(m_motionMagic.withPosition(0.0));
      } else {
          m_gripper.setControl(new DutyCycleOut(-p_gripperStowSpeed.getValue()));
          m_pivot.setControl(new DutyCycleOut(-p_pivotStowSpeed.getValue()));
  
          if (climberDebouncer.calculate(m_gripper.getVelocity().getValueAsDouble() > -1)
                && climberDebouncer.calculate(m_pivot.getVelocity().getValueAsDouble() > -1)) {
            calibrate();
            isCalibrated = true;
        }
    }
  }

  public double getPosition() {
    return m_pivot.getPosition().getValueAsDouble() * kPivotMotorRotationsToClimberPosition;
  }

  public void holdPostion() {
    m_gripper.setControl(new DutyCycleOut(0.0));
    m_pivot.setControl(new DutyCycleOut(0.0));
  }
  public void setPivot(double speed) {
    m_pivot.setControl(m_pivotRequest.withOutput(speed));
  }

  public void setGripperSpeed(double speed) {
    m_gripper.setControl(m_gripperRequest.withOutput(speed));
  }

  public void setGripperPosition(double position) {
    m_gripper.setControl(m_motionMagic.withPosition(position / kGripperMotorRotationsToPosition));
  }

  public void setPivotPosition(double position) {
    m_pivot.setControl(m_motionMagic.withPosition(position / kPivotMotorRotationsToClimberPosition));
  }

  public void calibrate() {
    m_gripper.setPosition(0.0);
    m_pivot.setPosition(0.0);
  }

  public Command stowFactory() {
    return new RunCommand(() -> stow(), this).beforeStarting(() -> climberDebouncer.calculate(false));
  }

  public Command holdPositionFactory() {
    return new RunCommand(() -> holdPostion(), this);
  }

  public Command pivotNeutralGrabberOpenFactory() {
    return new RunCommand(() -> pivotNeutralGrabberOpen(), this);
  }

  public Command pivotNeutralGrabberClosedFactory() {
    return new RunCommand(() -> pivotNeutralGrabberClosed(), this);
  }

  public Command pivotUpGrabberClosedFactory() {
    return new RunCommand(() -> pivotUpGrabberClosed(), this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("pivot position", getPosition());
  }
}
