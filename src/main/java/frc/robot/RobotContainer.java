// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.module.ModuleIOSim;
import frc.robot.subsystems.drive.module.SwerveMod;
import frc.robot.subsystems.drive.module.SwerveMod.ModuleName;

public class RobotContainer {
  private final PS5Controller driver = new PS5Controller(0);

  private final Drive m_drive;

  public RobotContainer() {
    if (Robot.isReal()) {
      m_drive = new Drive(null, null);
    } else {
      m_drive = new Drive(new SwerveMod[] {
          new SwerveMod(new ModuleIOSim(), ModuleName.FRONT_LEFT),
          new SwerveMod(new ModuleIOSim(), ModuleName.FRONT_RIGHT),
          new SwerveMod(new ModuleIOSim(), ModuleName.BACK_LEFT),
          new SwerveMod(new ModuleIOSim(), ModuleName.BACK_RIGHT),
      },
          new GyroIO() {
          });

    }

    configureBindings();
  }

  private void configureBindings() {

    m_drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            m_drive,
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX(),
            () -> -driver.getRightX(),
            () -> false
          )
        );

  }

  public void robotPeriodic() {
    RobotState.periodic(
        m_drive.getRawGyroRotation(),
        m_drive.getModulePositions()
      );
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
