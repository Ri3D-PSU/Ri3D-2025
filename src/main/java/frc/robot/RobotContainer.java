// Copyright 2021-2024 FRC 6328
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

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOSparkMax;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSparkMax;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSparkMax;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSparkMax;
import frc.robot.subsystems.vision.AprilTagVision;
import frc.robot.subsystems.vision.AprilTagVisionIOPhotonvision;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final AprilTagVision vision;
  private final Climber climber;
  private final Intake intake;
  private final Elevator elevator;

  private final double PROCESSOR_HEIGHT = 0;
  private final double SOURCE_HEIGHT = 8.75;
  private final double L1_HEIGHT = 3;
  private final double L2_HEIGHT = 5.5;
  private final double L3_HEIGHT = 21.5;
  private final double L4_HEIGHT = 52.5;
  private final double TOP_ALGAE_HEIGHT = 40;

  private final double PROCESSOR_ANGLE = 0;
  private final double SOURCE_ANGLE = 0.15;
  private final double L1_ANGLE = 0.3;
  private final double L2_ANGLE = 0.225;
  private final double L3_ANGLE = 0.225;
  private final double L4_ANGLE = 0.26;
  private final double TOP_ALGAE_ANGLE = 0;

  // Controller
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, IO devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL: // NAVX instead of pigeon
        // Real robot, instantiate hardware IO implementations
        vision = new AprilTagVision(new AprilTagVisionIOPhotonvision());
        climber = new Climber(new ClimberIOSparkMax());
        intake = new Intake(new IntakeIOSparkMax());
        elevator = new Elevator(new ElevatorIOSparkMax());
        drive =
            new Drive(
                new GyroIONavX(),
                new AprilTagVisionIOPhotonvision(),
                new ModuleIOSparkMax(0),
                new ModuleIOSparkMax(1),
                new ModuleIOSparkMax(2),
                new ModuleIOSparkMax(3));
        break;

      case SIM:
        vision = new AprilTagVision(new AprilTagVisionIOPhotonvision());
        climber = new Climber(new ClimberIO() {});
        intake = new Intake(new IntakeIO() {});
        elevator = new Elevator(new ElevatorIO() {});
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new AprilTagVisionIOPhotonvision(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        break;

      default:
        vision = new AprilTagVision(new AprilTagVisionIOPhotonvision());
        // Replayed robot, disable IO implementations
        climber = new Climber(new ClimberIO() {});
        intake = new Intake(new IntakeIO() {});
        elevator = new Elevator(new ElevatorIO() {});
        drive =
            new Drive(
                new GyroIO() {},
                new AprilTagVisionIOPhotonvision(),
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // Field centric swerve drive
    drive.setDefaultCommand(
        Drive.drive(
            drive,
            () -> driverController.getLeftY() * 0.6,
            () -> driverController.getLeftX() * 0.6,
            () -> -driverController.getRightX() * 0.65));

    // Slowed field centric swerve drive
    driverController
        .leftBumper()
        .whileTrue(
            Drive.drive(
                drive,
                () -> driverController.getLeftY() * 0.5,
                () -> driverController.getLeftX() * 0.5,
                () -> -driverController.getRightX() * 0.5));

    // Point wheels in x formation to stop
    driverController.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Point robot to april tag
    driverController
        .a()
        .whileTrue(
            Drive.driveToAprilTag(
                drive,
                vision,
                () -> driverController.getLeftY() * -1,
                () -> driverController.getLeftX(),
                () -> driverController.getRightX()));

    // Align robot to april tag
    driverController
        .y()
        .whileTrue(
            Drive.drive(
                drive,
                () -> vision.autoTranslateY(),
                () -> vision.autoTranslateX(),
                () -> -vision.autoRotate()));

    // Reset gyro
    driverController
        .start()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    // Climber command
    Command climbUpCommand =
        new StartEndCommand(() -> climber.setMotorVoltage(1.5), () -> climber.stopMotor(), climber);
    Command climbDownCommand =
        new StartEndCommand(() -> climber.setMotorVoltage(-4), () -> climber.stopMotor(), climber);
    Command climbHoldCommand =
        new StartEndCommand(
            () -> climber.setMotorVoltage(-1.5), () -> climber.stopMotor(), climber);

    driverController.povUp().whileTrue(climbUpCommand);
    driverController.povLeft().whileTrue(climbHoldCommand);
    driverController.povDown().whileTrue(climbDownCommand);

    // Eject algae
    Command ejectAlgaeCommand =
        new StartEndCommand(
            () -> intake.setAlgaeVoltage(12), () -> intake.setAlgaeVoltage(0), intake);
    driverController.rightBumper().whileTrue(ejectAlgaeCommand);

    Command intakeAlgaeCommand =
        new StartEndCommand(
            () -> intake.setAlgaeVoltage(-12), () -> intake.setAlgaeVoltage(0), intake);
    driverController.rightTrigger().whileTrue(intakeAlgaeCommand);

    // Intake coral
    Command intakeCoralCommand =
        new StartEndCommand(
            () -> intake.setCoralIntakeVoltage(-6), () -> intake.setCoralIntakeVoltage(0), intake);
    driverController.leftTrigger().whileTrue(intakeCoralCommand);

    Command ejectCoralCommand =
        new StartEndCommand(
            () -> intake.setCoralIntakeVoltage(6), () -> intake.setCoralIntakeVoltage(0), intake);
    operatorController.leftBumper().whileTrue(ejectCoralCommand);

    // Processor state
    Command liftToProcessorCommand =
        new RunCommand(() -> elevator.setPosition(PROCESSOR_HEIGHT), elevator);
    Command wristToProcessorCommand =
        new RunCommand(() -> intake.wristAngle(PROCESSOR_ANGLE), intake);
    ParallelCommandGroup processorCommandGroup =
        new ParallelCommandGroup(liftToProcessorCommand, wristToProcessorCommand);
    operatorController.povDown().onTrue(processorCommandGroup);

    // Source state
    Command liftToSourceCommand =
        new RunCommand(() -> elevator.setPosition(SOURCE_HEIGHT), elevator);
    Command wristToSourceCommand = new RunCommand(() -> intake.wristAngle(SOURCE_ANGLE), intake);
    ParallelCommandGroup sourceCommandGroup =
        new ParallelCommandGroup(liftToSourceCommand, wristToSourceCommand);
    operatorController.povLeft().onTrue(sourceCommandGroup);

    // L1 state
    Command liftToL1Command = new RunCommand(() -> elevator.setPosition(L1_HEIGHT), elevator);
    Command wristToL1Command = new RunCommand(() -> intake.wristAngle(L1_ANGLE), intake);
    ParallelCommandGroup l1CommandGroup =
        new ParallelCommandGroup(liftToL1Command, wristToL1Command);
    operatorController.a().onTrue(l1CommandGroup);

    // L2 state
    Command liftToL2Command = new RunCommand(() -> elevator.setPosition(L2_HEIGHT), elevator);
    Command wristToL2Command = new RunCommand(() -> intake.wristAngle(L2_ANGLE), intake);
    ParallelCommandGroup l2CommandGroup =
        new ParallelCommandGroup(liftToL2Command, wristToL2Command);
    operatorController.b().onTrue(l2CommandGroup);

    // L3 state
    Command liftToL3Command = new RunCommand(() -> elevator.setPosition(L3_HEIGHT), elevator);
    Command wristToL3Command = new RunCommand(() -> intake.wristAngle(L3_ANGLE), intake);
    ParallelCommandGroup l3CommandGroup =
        new ParallelCommandGroup(liftToL3Command, wristToL3Command);
    operatorController.y().onTrue(l3CommandGroup);

    // L4 state
    Command liftToL4Command = new RunCommand(() -> elevator.setPosition(L4_HEIGHT), elevator);
    Command wristToL4Command = new RunCommand(() -> intake.wristAngle(L4_ANGLE), intake);
    ParallelCommandGroup l4CommandGroup =
        new ParallelCommandGroup(liftToL4Command, wristToL4Command);
    operatorController.x().onTrue(l4CommandGroup);

    // Top algae state
    Command liftToTopAlgaeCommand =
        new RunCommand(() -> elevator.setPosition(TOP_ALGAE_HEIGHT), elevator);
    Command wristToTopAlgaeCommand =
        new RunCommand(() -> intake.wristAngle(TOP_ALGAE_ANGLE), intake);
    ParallelCommandGroup topAlgaeCommandGroup =
        new ParallelCommandGroup(liftToTopAlgaeCommand, wristToTopAlgaeCommand);
    operatorController.povUp().onTrue(topAlgaeCommandGroup);

    // Manual lift
    Command manualLift =
        new RunCommand(() -> elevator.setVoltage(-operatorController.getLeftY() * 0.5), elevator);
    // Command manualWrist =
    //     new RunCommand(() -> intake.setWristVoltage(operatorController.getRightY() * 0.25),
    // intake);
    // ParallelCommandGroup manualCommandGroup = new ParallelCommandGroup(manualLift, manualWrist);
    operatorController.start().whileTrue(manualLift);

    // Command combineOdometry =
    //     new RunCommand(() -> drive.updateOdometryWithVision(vision.getEstimatedPose()));

    // Trigger alwaysOnTrigger =
    //     new Trigger(
    //         () -> {
    //           return true;
    //         });

    // alwaysOnTrigger.whileTrue(combineOdometry);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new PathPlannerAuto("Example Auto");
  }
}
