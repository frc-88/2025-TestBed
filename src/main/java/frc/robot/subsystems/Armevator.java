package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.CommutationConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorArrangementValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;
import frc.robot.util.preferenceconstants.PIDPreferenceConstants;

public class Armevator extends SubsystemBase {
    private TalonFX m_elevatorMain = new TalonFX(Constants.ELEVATOR_MAIN_MOTOR, Constants.RIO_CANBUS);
    private TalonFX m_elevatorFollower = new TalonFX(Constants.ELEVATOR_FOLLOWER_MOTOR, Constants.RIO_CANBUS);
    private TalonFX m_arm = new TalonFX(Constants.ELEVATOR_ARM_MOTOR, Constants.RIO_CANBUS);
    private TalonFXS m_manipulator = new  TalonFXS(Constants.ELEVATOR_MANIPULATOR_MOTOR, Constants.RIO_CANBUS);


    private final CANrange m_canRangeLeft = new CANrange(Constants.ARM_LEFT_CANRANGE, Constants.RIO_CANBUS);
    private final CANrange m_canRangeMiddle = new CANrange(Constants.ARM_MIDDLE_CANRANGE, Constants.RIO_CANBUS);
    private final CANrange m_canRangeRight = new CANrange(Constants.ARM_RIGHT_CANRANGE, Constants.RIO_CANBUS);

    private final Debouncer elevatorDebouncer = new Debouncer(1.0);

    private PIDPreferenceConstants elevatorPID = new PIDPreferenceConstants("Armevator/Elevator/PID");
    private DoublePreferenceConstant p_elevatorMaxVelocity = new DoublePreferenceConstant(
            "Armevator/Elevator/MotionMagicVelocity", 0.0);
    private DoublePreferenceConstant p_elevatorMaxAcceleration = new DoublePreferenceConstant(
            "Armevator/Elevator/MotionMagicAcceleration", 0.0);
    private DoublePreferenceConstant p_elevatorJerk = new DoublePreferenceConstant("Armevator/Elevator/MotionMagicJerk",
            0.0);
    private DoublePreferenceConstant p_elevatorTargetInches = new DoublePreferenceConstant(
            "Armevator/Elevator/TargetPositionInches", 0.0);

    private PIDPreferenceConstants armPID = new PIDPreferenceConstants("Armevator/Arm/PID");
    private DoublePreferenceConstant p_armMaxVelocity = new DoublePreferenceConstant(
            "Armevator/Arm/MotionMagicVelocity", 0.0);
    private DoublePreferenceConstant p_armMaxAcceleration = new DoublePreferenceConstant(
            "Armevator/Arm/MotionMagicAcceleration", 0.0);
    private DoublePreferenceConstant p_armJerk = new DoublePreferenceConstant("Armevator/Arm/MotionMagicJerk", 0.0);
    private DoublePreferenceConstant p_armTargetDegrees = new DoublePreferenceConstant(
            "Armevator/Arm/TargetPositionDegrees", 0.0);


            private DoublePreferenceConstant p_manipulatorInSpeed = new DoublePreferenceConstant("Armevator/Manipultor/InSpeed", 0.3);
            private DoublePreferenceConstant p_manipulatorOutSpeed = new DoublePreferenceConstant("Armevator/Manipultor/OutSpeed", 0.3);
            private DoublePreferenceConstant p_manipulatorCurrentLimit = new DoublePreferenceConstant("Armevator/Manipultor/CurrentLimit", 20);

    private MotionMagicVoltage motionmagicrequest = new MotionMagicVoltage(0.0);

    public Armevator() {
        configureTalons();

        CANrangeConfiguration canRangemiddlecfg = new CANrangeConfiguration();
        CANrangeConfiguration canRangeleftcfg = new CANrangeConfiguration();
        CANrangeConfiguration canRangerightcfg = new CANrangeConfiguration();
        canRangemiddlecfg.FovParams.FOVRangeX = 6.5;
        canRangeleftcfg.FovParams.FOVRangeX = 6.5;
        canRangerightcfg.FovParams.FOVRangeX = 6.5;
        canRangemiddlecfg.FovParams.FOVRangeY = 27.0;
        canRangeleftcfg.FovParams.FOVRangeY = 27.0;
        canRangerightcfg.FovParams.FOVRangeY = 27.0;

        canRangeleftcfg.ToFParams.UpdateFrequency = 50;
        canRangerightcfg.ToFParams.UpdateFrequency = 50;
        canRangemiddlecfg.ToFParams.UpdateFrequency = 50;

        canRangeleftcfg.ProximityParams.ProximityThreshold = 0.5;
        canRangemiddlecfg.ProximityParams.ProximityThreshold = 0.5;
        canRangerightcfg.ProximityParams.ProximityThreshold = 0.5;

        m_canRangeLeft.getConfigurator().apply(canRangeleftcfg);
        m_canRangeRight.getConfigurator().apply(canRangerightcfg);
        m_canRangeMiddle.getConfigurator().apply(canRangemiddlecfg);
    }

    public void configureTalons() {
        TalonFXConfiguration maincfg = new TalonFXConfiguration();
        TalonFXConfiguration followercfg = new TalonFXConfiguration();
        TalonFXConfiguration armcfg = new TalonFXConfiguration();
        
        TalonFXSConfiguration manipulatorConfiguration = new TalonFXSConfiguration();
        manipulatorConfiguration.CurrentLimits.SupplyCurrentLimit = p_manipulatorCurrentLimit.getValue();
        manipulatorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        manipulatorConfiguration.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
        manipulatorConfiguration.OpenLoopRamps = new OpenLoopRampsConfigs().withDutyCycleOpenLoopRampPeriod(0);
        m_manipulator.getConfigurator().apply(manipulatorConfiguration);

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

        armcfg.MotionMagic.MotionMagicCruiseVelocity = p_armMaxVelocity.getValue();
        armcfg.MotionMagic.MotionMagicAcceleration = p_armMaxAcceleration.getValue();
        armcfg.MotionMagic.MotionMagicJerk = p_armJerk.getValue();

        m_elevatorMain.getConfigurator().apply(maincfg);
        m_elevatorFollower.getConfigurator().apply(followercfg);
        m_arm.getConfigurator().apply(armcfg);

        m_elevatorFollower.setControl(new Follower(Constants.ELEVATOR_MAIN_MOTOR, false));
    }

    public void elevatorSetPosition(double position) {
        m_elevatorMain.setControl(motionmagicrequest.withPosition(position / Constants.ELEVATOR_ROTATIONS_TO_INCHES));
    }

    public void armSetPosition() {
        m_arm.setControl(
                motionmagicrequest.withPosition(p_armTargetDegrees.getValue() / Constants.ARM_ROTATIONS_TO_DEGREES));
    }

    public void elevatorStop() {
        m_elevatorMain.setControl(new DutyCycleOut(0.0));
    }

    public void armStop() {
        m_arm.setControl(new DutyCycleOut(0.0));
    }
    public void manipulatorStop(){
        m_manipulator.setControl(new DutyCycleOut(0.0));
    }

    public void manipulatorIn(){
        m_manipulator.setControl(new DutyCycleOut(p_manipulatorInSpeed.getValue()));
    }

    public void manipulatorOut(){
        m_manipulator.setControl(new DutyCycleOut(p_manipulatorOutSpeed.getValue()));
    }

    public void elevatorCalibrate() {
        m_elevatorMain.setPosition(0.0);
    }

    public void armCalibrate() {
        m_arm.setPosition(0.0);
    }

    public void elevatorSetSlowSpeed() {
        m_elevatorMain.setControl(new DutyCycleOut(0.1));
    }

    public void armSetSlowSpeed() {
        m_arm.setControl(new DutyCycleOut(0.1));
    }

    public void elevatorSetCalibrateSpeed() {
        m_elevatorMain.setControl(new DutyCycleOut(-0.09));
    }

    public void armSetSpeed() {
        m_arm.setControl(new DutyCycleOut(-0.09));
    }

    public Command calibrateArmFactory() {
        return new InstantCommand(() -> armCalibrate(), this);
    }

    public Command calibrateElevatorFactory() {
        return new RunCommand(() -> elevatorSetCalibrateSpeed(), this)
                .until(() -> elevatorDebouncer
                        .calculate(Math.abs(m_elevatorMain.getVelocity().getValueAsDouble()) < 0.02))
                .andThen(() -> {
                    elevatorStop();
                    elevatorCalibrate();
                })
                .beforeStarting(() -> elevatorDebouncer.calculate(false));
    }
    
    public Command manipulatorOutFactory() {
        return new RunCommand(() -> manipulatorOut(), this) 
                .withTimeout(1.0)
                .andThen(()-> manipulatorStop() );
    }

    public Command manipulatorInFactory() {
        return new RunCommand(() -> manipulatorIn(), this) 
        .until(()-> m_canRangeMiddle.getIsDetected().getValue())
        .andThen(()-> manipulatorStop())  ;    
    }

    public Command manipulatorStopFactory() {
        return new InstantCommand(() -> manipulatorStop(), this) ;  
    }


    public Command stopElevatorFactory() {
        return new RunCommand(() -> elevatorStop(), this);
    }

    public Command setElevatorSlowSpeedFactory() {
        return new RunCommand(() -> elevatorSetSlowSpeed(), this);
    }

    public Command setElevatorPostionFactory() {
        return new RunCommand(() -> elevatorSetPosition(p_elevatorTargetInches.getValue()), this);
    }

    public Command stopArmFactory() {
        return new RunCommand(() -> armStop(), this);
    }

    public Command setArmSlowSpeedFactory() {
        return new RunCommand(() -> armSetSlowSpeed(), this);
    }

    public Command setArmPostionFactory() {
        return new RunCommand(() -> elevatorSetPosition(p_armTargetDegrees.getValue()), this);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator Positon", m_elevatorMain.getPosition().getValueAsDouble() * Constants.ELEVATOR_ROTATIONS_TO_INCHES);
        SmartDashboard.putNumber("Arm Position", m_arm.getPosition().getValueAsDouble() * Constants.ARM_ROTATIONS_TO_DEGREES);
        SmartDashboard.putNumber("CAN Range Left Distance", Units.metersToInches(m_canRangeLeft.getDistance().getValueAsDouble()));
        SmartDashboard.putNumber("CAN Range Middle Distance", Units.metersToInches(m_canRangeMiddle.getDistance().getValueAsDouble()));
        SmartDashboard.putNumber("CAN Range Right Distance", Units.metersToInches(m_canRangeRight.getDistance().getValueAsDouble()));
        SmartDashboard.putNumber("Armevator Setpoint", m_elevatorMain.getClosedLoopReference().getValueAsDouble());
        SmartDashboard.putNumber("Armevator Velocity", m_elevatorMain.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Arm Setpoint", m_arm.getClosedLoopReference().getValueAsDouble());
        SmartDashboard.putNumber("Arm Velocity", m_arm.getVelocity().getValueAsDouble());
    }
}
