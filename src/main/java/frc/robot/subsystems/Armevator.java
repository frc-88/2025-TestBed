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

public class Armevator  extends SubsystemBase {

    private TalonFX m_elevatorFollower = new TalonFX(Constants.ELEVATOR_FOLLOWER_MOTOR, "rio");
    private TalonFX m_elevatorMain = new TalonFX(Constants.ELEVATOR_MAIN_MOTOR, "rio");
    private TalonFX m_arm = new TalonFX(Constants.ELEVATOR_ARM_MOTOR, "rio");

    private final Debouncer elevatorDebouncer = new Debouncer(1.0);

    private PIDPreferenceConstants elevatorPID = new PIDPreferenceConstants("Elevator/MainMotorPID");
    private PIDPreferenceConstants armPID = new PIDPreferenceConstants("Armevator/Elevator/ArmMotorPID");
    private DoublePreferenceConstant p_requestInches = new DoublePreferenceConstant("Armevator/Elevator/TargetPositionInches", 0.0);
    private DoublePreferenceConstant p_elevatorMaxVelocity = new DoublePreferenceConstant("Armevator/Elevator/MotionMagicVelocity", 0.0);
    private DoublePreferenceConstant p_elevatorMaxAcceleration = new DoublePreferenceConstant("Armevator/Elevator/MotionMagicAcceleration", 0.0);
    private DoublePreferenceConstant p_elevatorJerk = new DoublePreferenceConstant("Armevator/Elevator/MotionMagicJerk", 0.0);
    private DoublePreferenceConstant p_ArmMaxVelocity = new DoublePreferenceConstant("Armevator/Arm/MotionMagicVelocity", 0.0);
    private DoublePreferenceConstant p_ArmMaxAcceleration = new DoublePreferenceConstant("Armevator/Arm/MotionMagicAcceleration", 0.0);
    private DoublePreferenceConstant p_ArmJerk = new DoublePreferenceConstant("Armevator/Arm/MotionMagicJerk", 0.0);

    private MotionMagicVoltage motionmagicrequest = new MotionMagicVoltage(0.0);
    
    public Armevator() {
        configureTalons();
    }

    public void configureTalons() {
        TalonFXConfiguration maincfg = new TalonFXConfiguration();
        TalonFXConfiguration followercfg = new TalonFXConfiguration();
        TalonFXConfiguration armcfg = new TalonFXConfiguration();

        maincfg.Slot0.kP = elevatorPID.getKP().getValue();
        maincfg.Slot0.kI = elevatorPID.getKI().getValue();
        maincfg.Slot0.kD = elevatorPID.getKD().getValue();
        maincfg.Slot0.kV = elevatorPID.getKF().getValue();
        
        maincfg.MotionMagic.MotionMagicCruiseVelocity = p_elevatorMaxVelocity.getValue();
        maincfg.MotionMagic.MotionMagicAcceleration = p_elevatorMaxAcceleration.getValue();
        maincfg.MotionMagic.MotionMagicJerk = p_elevatorJerk.getValue();

        armcfg.Slot0.kP = armPID.getKP().getValue();
        armcfg.Slot0.kI = armPID.getKI().getValue();
        armcfg.Slot0.kD = armPID.getKD().getValue();
        armcfg.Slot0.kV = armPID.getKF().getValue();

        armcfg.MotionMagic.MotionMagicCruiseVelocity = p_ArmMaxVelocity.getValue();
        armcfg.MotionMagic.MotionMagicAcceleration = p_ArmMaxAcceleration.getValue();
        armcfg.MotionMagic.MotionMagicJerk = p_ArmJerk.getValue();

        m_elevatorMain.getConfigurator().apply(maincfg);
        m_elevatorFollower.getConfigurator().apply(followercfg);
        m_arm.getConfigurator().apply(armcfg);

        m_elevatorFollower.setControl(new Follower(Constants.ELEVATOR_MAIN_MOTOR, false));
    }

    public void elevatorSetPosition() {
        m_elevatorMain.setControl(motionmagicrequest.withPosition(p_requestInches.getValue() / Constants.ELEVATOR_ROTATIONS_TO_INCHES).withFeedForward(0.1));
    }
    public void armSetPosition() {
        m_arm.setControl(motionmagicrequest.withPosition(p_requestInches.getValue() / Constants.ELEVATOR_ROTATIONS_TO_INCHES).withFeedForward(0.1));
    }

    public void elevatorStop() {
        m_elevatorMain.setControl(new DutyCycleOut(0.0));
    }
    public void armStop() {
        m_arm.setControl(new DutyCycleOut(0.0));
    }

    public void elevatorCalibrate() {
        m_elevatorMain.setPosition(0.0);
        //sm_back.setPosition(0.0);
    }
    public void armCalibrate() {
        m_arm.setPosition(0.0);
        //sm_back.setPosition(0.0);
    }

    public void elevatorSetSlowSpeed() {
        m_elevatorMain.setControl(new DutyCycleOut(0.1));
    }
    public void armSetSlowSpeed() {
        m_arm.setControl(new DutyCycleOut(0.1));
    }

    public void elevatorSetSpeed() {
        m_elevatorMain.setControl(new DutyCycleOut(-0.09));
    }
    public void armSetSpeed() {
        m_arm.setControl(new DutyCycleOut(-0.09));
    }

    // public Command calibrateFactory() {
    //     return new InstantCommand(() -> elevatorCalibrate(), this);
    // }
    public Command calibrateArmFactory() {
        return new InstantCommand(() -> armCalibrate(), this);
    }

    public Command calibrateElevatorFactory() {
        return new RunCommand(() -> elevatorSetSpeed(), this)
        .until(() -> elevatorDebouncer.calculate(Math.abs(m_elevatorMain.getVelocity().getValueAsDouble())  < 0.02))
        .andThen(() -> {
            elevatorStop();
            elevatorCalibrate();
        })
        .beforeStarting(() -> elevatorDebouncer.calculate(false));
    }

    public Command stopFactory() {
        return new RunCommand(() -> elevatorStop(), this);
    }

    public Command setSlowSpeedFactory() {
        return new RunCommand(() -> elevatorSetSlowSpeed(), this);
    }
    public Command setPostionFactory() {
        return new RunCommand(() -> elevatorSetPosition(), this);
    }    

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator Main Positon", m_elevatorMain.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Elevator Follower Positon", m_elevatorFollower.getPosition().getValueAsDouble());
    }
}
 