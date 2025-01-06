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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSparkMax;
import frc.robot.subsystems.sysid.CreateSysIdCommand;
import frc.robot.subsystems.vision.AprilTagVision;
import frc.robot.subsystems.vision.AprilTagVisionIOPhotonvision;

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

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private static SendableChooser<Command> autoChooser;
  private static SendableChooser<Command> sysidChooser = new SendableChooser<Command>();

  static boolean sysIdInit = false;

  /** The container for the robot. Contains subsystems, IO devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL: // NAVX instead of pigeon
        // Real robot, instantiate hardware IO implementations
        vision = new AprilTagVision(new AprilTagVisionIOPhotonvision());
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

    // add pathplanner autochooser
    autoChooser = AutoBuilder.buildAutoChooser();
    ShuffleboardTab autoTab = Shuffleboard.getTab("Auto");
    autoTab.add(autoChooser).withSize(5, 5).withPosition(1, 1);

    ShuffleboardTab sysidTab = Shuffleboard.getTab("Sysid");
    sysidChooser.setDefaultOption("None!", new WaitCommand(0.1));
    // sysidChooser.addOption("AnglerSysID",
    // CreateSysidCommand.createCommand(targetingSubsystem::sysIdQuasistatic,
    // targetingSubsystem::sysIdDynamic, "AnglerSysID", () -> controller.getHID().getAButton(), ()
    // -> controller.getHID().getBButton()));
    sysidChooser.addOption(
        "SwerveSysID",
        CreateSysIdCommand.createCommand(
            drive::sysIdQuasistatic,
            drive::sysIdDynamic,
            "SwerveSysId",
            controller,
            () -> drive.stop()));

    // sysidChooser.addOption("ShooterSysID",
    //     CreateSysidCommand.createCommand(
    //         shooterSubsystem::sysIdQuasistatic,
    //         shooterSubsystem::sysIdDynamic,
    //         "ShooterSysID", () -> controller.getHID().getAButton(), () ->
    // controller.getHID().getBButton()));
    sysidTab.add(sysidChooser).withSize(5, 5).withPosition(1, 1);
    sysIdInit = true;

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
    drive.setDefaultCommand(
        Drive.drive(
            drive,
            () -> controller.getLeftY(),
            () -> controller.getLeftX(),
            () -> -controller.getRightX()));
    controller
        .leftBumper()
        .whileTrue(
            Drive.drive(
                drive,
                () -> controller.getLeftY() * 0.1,
                () -> controller.getLeftX() * 0.1,
                () -> -controller.getRightX() * 0.25));
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));
    controller
        .a()
        .whileTrue(
            Drive.drive(
                drive,
                () -> controller.getLeftY(),
                () -> controller.getLeftX(),
                () -> -vision.autoRotate()));
    controller
        .start()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));
    controller
        .y()
        .whileTrue(
            Drive.drive(
                drive,
                () -> vision.autoTranslateY(),
                () -> vision.autoTranslateX(),
                () -> -vision.autoRotate()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public static Command getSelectedSysid() {
    return sysidChooser.getSelected();
  }
}
