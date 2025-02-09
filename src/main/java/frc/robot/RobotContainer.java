// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Doghouse;
import frc.robot.subsystems.Armevator;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public Climber climber = new Climber();

    private Trigger onDisable = new Trigger(() -> RobotState.isDisabled() && climber.getPositionGasMotor() < 70.0);

    public Armevator m_armevator = new Armevator();

    public Doghouse m_doghouse = new Doghouse();
    //public Trigger stop = new Trigger(() -> RobotState.isDisabled() && climber.getPositionGasMotor() < 5.0);

    public RobotContainer() {
        configureBindings();
        configureButtons();
    }

    public void configureButtons() {
        SmartDashboard.putData("PivotNeutralGrabberOpen", climber.pivotNeutralGrabberOpenFactory());
        SmartDashboard.putData("PivotNeutralGrabberClosed", climber.pivotNeutralGrabberClosedFactory());
        SmartDashboard.putData("PivotUpGrabberClosed", climber.pivotUpGrabberClosedFactory());
        SmartDashboard.putData("GasMotorRotations", climber.runGasMotorRotationsFactory());
        SmartDashboard.putData("StopGasMotor", climber.stopGasMotorFactory());
        SmartDashboard.putData("CalibrateGasMotor", climber.calibrateGasMotorFactory().ignoringDisable(true));
        SmartDashboard.putData("SetPositionInches", climber.setPositionFactory());
        SmartDashboard.putData("Calibrate Encoder", climber.calibrateEncoderFactory().ignoringDisable(true));
        SmartDashboard.putData("Set Coast", climber.setNeutralModeFactory().ignoringDisable(true));
        SmartDashboard.putData("Set Brake", climber.gasMotorBrakeModeFactory().ignoringDisable(true));
        SmartDashboard.putData("Prep Climber", climber.prepClimber());

        SmartDashboard.putData("Calibrate Elevator", m_armevator.calibrateElevatorFactory());
        SmartDashboard.putData("Calibrate Arm", m_armevator.calibrateArmFactory());
        SmartDashboard.putData("Set Position Elevator", m_armevator.setElevatorPostionFactory());
        SmartDashboard.putData("Slow Speed Elevator", m_armevator.setElevatorSlowSpeedFactory());
        SmartDashboard.putData("Stop Elevator", m_armevator.stopElevatorFactory());
        SmartDashboard.putData("Set Position Arm", m_armevator.setArmPostionFactory());
        SmartDashboard.putData("Arm Go To Zero", m_armevator.armGoToZeroFactory());
        SmartDashboard.putData("Slow Speed Arm", m_armevator.setArmSlowSpeedFactory());
        SmartDashboard.putData("Stop Arm", m_armevator.stopArmFactory());
        SmartDashboard.putData("Out Manipulator",m_armevator.manipulatorOutFactory());
        SmartDashboard.putData("In Manipulator",m_armevator.manipulatorInFactory());
        SmartDashboard.putData("Stop Manipulator",m_armevator.manipulatorStopFactory());
        SmartDashboard.putData("Go To One Inch",m_armevator.goToOneInchFactory());
        SmartDashboard.putData("Go To Tilt Angle", m_armevator.goToTiltAngleFactory());

        SmartDashboard.putData("Stop Doghouse", m_doghouse.stopMovingFactory());
        SmartDashboard.putData("Slow Doghouse", m_doghouse.moveSlowFactory());
        SmartDashboard.putData("Fast Doghouse", new ParallelCommandGroup(m_doghouse.moveFastFactory(),
         m_armevator.manipulatorInFactory()
         .andThen(m_armevator.goToTiltAngleFactory())
         .andThen(m_armevator.backUpFactory())));

        SmartDashboard.putData("L4", m_armevator.L4Factory());
        SmartDashboard.putData("L3", m_armevator.L3Factory());
        SmartDashboard.putData("L2", m_armevator.L2Factory());
    }

    private void configureBindings() {
        m_armevator.setDefaultCommand(m_armevator.defaultCommand());
        climber.shouldBrake().onTrue(climber.gasMotorBrakeModeFactory().ignoringDisable(true))
                .onFalse(climber.setNeutralModeFactory().ignoringDisable(true));
        climber.shouldGripperClose().onTrue(climber.pivotNeutralGrabberClosedFactory());
        // onDisable.whileTrue(climber.gasMotorBrakeModeFactory()).onTrue(climber.gasMotorBrakeModeFactory());
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        // stop.onTrue(climber.gasMotorBrakeModeFactory());
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                        .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public void teleopInit() {
        climber.shouldBrake().onTrue(climber.gasMotorBrakeModeFactory().ignoringDisable(true));
    }

    public void disableInit() {
        // climber.gasMotorNeutralMode();
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
