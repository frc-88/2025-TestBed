package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;
import frc.robot.util.preferenceconstants.PIDPreferenceConstants;

public class Elevator  extends SubsystemBase {

    private TalonFX m_elevatorFollower = new TalonFX(Constants.ELEVATOR_FOLLOWER_MOTOR, "rio");
    private TalonFX m_elevatorMain = new TalonFX(Constants.ELEVATOR_MAIN_MOTOR, "rio");
    //private TalonFX m_pivot = new TalonFX(Constants.ELEVATOR_PIVOT_MOTOR, "rio");

    private final Debouncer elevatorDebouncer = new Debouncer(1.0);

    private PIDPreferenceConstants elevatorPID = new PIDPreferenceConstants("Elevator/MainMotorPID");
    private PIDPreferenceConstants pivotPID = new PIDPreferenceConstants("Elevator/PivotMotorPID");
    private DoublePreferenceConstant p_requestInches = new DoublePreferenceConstant("Elevator/TargetPositionInches", 0.0);
    private DoublePreferenceConstant p_velocity = new DoublePreferenceConstant("Elevator/MotionMagicVelocity", 0.0);
    private DoublePreferenceConstant p_acceleration = new DoublePreferenceConstant("Elevator/MotionMagicAcceleration", 0.0);
    private DoublePreferenceConstant p_jerk = new DoublePreferenceConstant("Elevator/MotionMagicJerk", 0.0);
    private MotionMagicVoltage motionmagicrequest = new MotionMagicVoltage(0.0);
    
    public Elevator() {
        configureTalons();
    }

    public void configureTalons() {
        TalonFXConfiguration maincfg = new TalonFXConfiguration();
        TalonFXConfiguration followercfg = new TalonFXConfiguration();
        TalonFXConfiguration pivotcfg = new TalonFXConfiguration();

        maincfg.Slot0.kP = elevatorPID.getKP().getValue();
        maincfg.Slot0.kI = elevatorPID.getKI().getValue();
        maincfg.Slot0.kD = elevatorPID.getKD().getValue();
        maincfg.Slot0.kV = elevatorPID.getKF().getValue();
        maincfg.MotionMagic.MotionMagicCruiseVelocity = p_velocity.getValue();
        maincfg.MotionMagic.MotionMagicAcceleration = p_acceleration.getValue();
        maincfg.MotionMagic.MotionMagicJerk = p_jerk.getValue();

        pivotcfg.Slot0.kP = pivotPID.getKP().getValue();
        pivotcfg.Slot0.kI = pivotPID.getKI().getValue();
        pivotcfg.Slot0.kD = pivotPID.getKD().getValue();

        m_elevatorMain.getConfigurator().apply(maincfg);
        m_elevatorFollower.getConfigurator().apply(followercfg);
        // m_pivot.getConfigurator().apply(pivotcfg);

        m_elevatorFollower.setControl(new Follower(Constants.ELEVATOR_MAIN_MOTOR, false));
    }

    public void setPosition() {
        m_elevatorMain.setControl(motionmagicrequest.withPosition(p_requestInches.getValue() / Constants.ELEVATOR_ROTATIONS_TO_INCHES).withFeedForward(0.1));
    }

    public void stop() {
        m_elevatorMain.setControl(new DutyCycleOut(0.0));
    }

    public void calibrate() {
        m_elevatorMain.setPosition(0.0);
        //sm_back.setPosition(0.0);
    }

    public void setSlowSpeed() {
        m_elevatorMain.setControl(new DutyCycleOut(0.1));
    }

    public void setSpeed() {
        m_elevatorMain.setControl(new DutyCycleOut(-0.09));
    }

    public Command calibrateFactory() {
        return new InstantCommand(() -> calibrate(), this);
    }

    public Command calibrateElevatorFactory() {
        return new RunCommand(() -> setSpeed(), this)
        .until(() -> elevatorDebouncer.calculate(Math.abs(m_elevatorMain.getVelocity().getValueAsDouble())  < 0.02))
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
        SmartDashboard.putNumber("Elevator Main Positon", m_elevatorMain.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Elevator Follower Positon", m_elevatorFollower.getPosition().getValueAsDouble());
    }
}
 