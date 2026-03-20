// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.*;
import frc.robot.commands.*;


public class RobotContainer {

  public final DriveSubsystem drive;
  public final Intake intake = new Intake();
  public final Launcher launcher = new Launcher();
  public final Climber climber = new Climber();
  public final LEDSubsystem led = new LEDSubsystem();
  private final CommandXboxController controller = new CommandXboxController(0);
  private Boolean fieldRelative = true;
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  public RobotContainer() {

    // PathPlanner Named Commands
    new EventTrigger("Start Intake").onTrue(intake.fire());
    new EventTrigger("Stop Intake").onTrue(intake.stop());
    new EventTrigger("Start Firing").onTrue(launcher.fire());
    new EventTrigger("Stop Firing").onTrue(launcher.stop());
    new EventTrigger("Dump").onTrue(intake.dump());
    new EventTrigger("Raise Climber").onTrue(climber.raise());
    new EventTrigger("Lower Climber").onTrue(climber.lower());
    


    // Instantiate Drive
    drive = new DriveSubsystem();
    
    // Auto Chooser
    autoChooser.setDefaultOption("Do Nothing", Commands.none());
    autoChooser.addOption("Left Trench", new PathPlannerAuto("AutoLeftSide"));
    autoChooser.addOption("Left Trench Extended", new PathPlannerAuto("AutoLeftSideExtended"));
    autoChooser.addOption("Right Trench", new PathPlannerAuto("AutoRightSide"));
    autoChooser.addOption("Right Trench Extended", new PathPlannerAuto("AutoRightSideExtended"));
    autoChooser.addOption("Left of Center", new PathPlannerAuto("AutoCenterLeft"));
    autoChooser.addOption("Right of Center", new PathPlannerAuto("AutoCenterRight"));
    SmartDashboard.putData("Auto Chooser", autoChooser);

    configureBindings();
    drive.setDefaultCommand(drive.driveCommand(controller, () -> fieldRelative));        
  }

  private void configureBindings() {
    DoubleSupplier fwd = () -> edu.wpi.first.math.MathUtil.applyDeadband(controller.getLeftY() * DriveSubsystem.kSpeedLimit, DriveSubsystem.kControllerDeadband);
    DoubleSupplier str = () -> edu.wpi.first.math.MathUtil.applyDeadband(controller.getLeftX() * DriveSubsystem.kSpeedLimit, DriveSubsystem.kControllerDeadband);

    
    
    controller.leftTrigger().whileTrue(intake.fire());
    controller.leftBumper().whileTrue( Commands.parallel(launcher.dump(), intake.dump()));
    controller.povUp().onTrue(climber.raise()).onFalse(climber.stop());
    controller.povDown().onTrue(climber.lower()).onFalse(climber.stop());
    controller.povRight().whileTrue(launcher.increaseLaunchVoltage());
    controller.povLeft().whileTrue(launcher.decreaseLaunchVoltage());
    controller.rightTrigger().whileTrue(Commands.parallel(launcher.fire(), intake.fire()));
    controller.rightBumper().whileTrue(new AimAtTargetCommand(drive, launcher, fwd, str, ()-> fieldRelative));
    controller.y().onTrue(Commands.runOnce(drive::setPoseFromVision));       
    controller.start().onTrue(new InstantCommand(() -> fieldRelative = !fieldRelative));
    controller.x().onTrue(Commands.runOnce(() -> launcher.setUseRpmControl(!launcher.isUsingRpmControl())
));
  }

  public Command getAutonomousCommand() {
      return autoChooser.getSelected();
  }


}
