package frc.robot.subsystems;

import java.util.function.DoublePredicate;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;
import frc.robot.util.preferenceconstants.PIDPreferenceConstants;

public class Elevator  extends SubsystemBase {

    private TalonFX m_back = new TalonFX(Constants.ELEVATOR_BACK_MOTOR, "rio");
    private TalonFX m_front = new TalonFX(Constants.ELEVATOR_FRONT_MOTOR, "rio");
    //private TalonFX m_pivot = new TalonFX(Constants.ELEVATOR_PIVOT_MOTOR, "rio");

    private final Debouncer elevatorDebouncer = new Debouncer(1.0);

    private PIDPreferenceConstants backPID = new PIDPreferenceConstants("Elevator/BackMotorPID");
    private PIDPreferenceConstants frontPID = new PIDPreferenceConstants("Elevator/FrontMotorPID");
    private PIDPreferenceConstants pivotPID = new PIDPreferenceConstants("Elevator/PivotMotorPID");
    private DoublePreferenceConstant p_requestInches = new DoublePreferenceConstant("Elevator/TargetPositionInches", 0.0);
    private DoublePreferenceConstant p_velocity = new DoublePreferenceConstant("Elevator/MotionMagicVelocity", 0.0);
    private DoublePreferenceConstant p_acceleration = new DoublePreferenceConstant("Elevator/MotionMagicAcceleration", 0.0);
    private DoublePreferenceConstant p_jerk = new DoublePreferenceConstant("Elevator/MotionMagicJerk", 0.0);
    private PositionVoltage request = new PositionVoltage(0.0);
    private MotionMagicDutyCycle motionmagicrequest = new MotionMagicDutyCycle(0.0);
    
    public Elevator() {
        configureTalons();
    }

    public void configureTalons() {
        TalonFXConfiguration backcfg = new TalonFXConfiguration();
        TalonFXConfiguration frontcfg = new TalonFXConfiguration();
        TalonFXConfiguration pivotcfg = new TalonFXConfiguration();

        backcfg.Slot0.kP = backPID.getKP().getValue();
        backcfg.Slot0.kI = backPID.getKI().getValue();
        backcfg.Slot0.kD = backPID.getKD().getValue();

        frontcfg.Slot0.kP = frontPID.getKP().getValue();
        frontcfg.Slot0.kI = frontPID.getKI().getValue();
        frontcfg.Slot0.kD = frontPID.getKD().getValue();
        frontcfg.Slot0.kV = frontPID.getKF().getValue();
        frontcfg.MotionMagic.MotionMagicCruiseVelocity = p_velocity.getValue();
        frontcfg.MotionMagic.MotionMagicAcceleration = p_acceleration.getValue();
        frontcfg.MotionMagic.MotionMagicJerk = p_jerk.getValue();

        pivotcfg.Slot0.kP = pivotPID.getKP().getValue();
        pivotcfg.Slot0.kI = pivotPID.getKI().getValue();
        pivotcfg.Slot0.kD = pivotPID.getKD().getValue();

        m_front.getConfigurator().apply(frontcfg);
        m_back.getConfigurator().apply(backcfg);
        //m_pivot.getConfigurator().apply(pivotcfg);

        m_back.setControl(new Follower(Constants.ELEVATOR_FRONT_MOTOR, false));
    }

    public void setPosition() {
        m_front.setControl(motionmagicrequest.withPosition(p_requestInches.getValue() / Constants.ELEVATOR_ROTATIONS_TO_INCHES).withFeedForward(0.1));
    }

    public void stop() {
        m_front.setControl(new DutyCycleOut(0.0));
    }

    public void calibrate() {
        m_front.setPosition(0.0);
        //sm_back.setPosition(0.0);
    }

    public void setSlowSpeed() {
        m_front.setControl(new DutyCycleOut(0.1));
    }

    public void setSpeed() {
        m_front.setControl(new DutyCycleOut(-0.09));
    }

    public Command calibrateFactory() {
        return new InstantCommand(() -> calibrate(), this);
    }

    public Command calibrateElevator() {
        return new RunCommand(() -> setSpeed(), this)
        .until(() -> elevatorDebouncer.calculate(Math.abs(m_front.getVelocity().getValueAsDouble())  < 0.02))
        .andThen(() -> {
            stop();
            calibrate();
        })
        .beforeStarting(() -> elevatorDebouncer.calculate(false));
    }

    public Command stopFactory() {
        return new RunCommand(() -> stop(), this);
    }

    public Command setSlowSpeedFactory() {
        return new RunCommand(() -> setSlowSpeed(), this);
    }
    public Command setPostionFactory() {
        return new RunCommand(() -> setPosition(), this);
    }    

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator Front Positon", m_front.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Elevator Back Positon", m_back.getPosition().getValueAsDouble());
    }
}
 