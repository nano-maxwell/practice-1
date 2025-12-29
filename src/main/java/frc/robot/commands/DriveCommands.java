package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;

public final class DriveCommands {

    public static final Command joystickDrive(
            Drive drive,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier rotationSupplier,
            BooleanSupplier isRobotRelative) {
        return Commands.run(
                () -> {
                    double x = xSupplier.getAsDouble();
                    double y = ySupplier.getAsDouble();
                    double rot = rotationSupplier.getAsDouble();

                    x = Math.abs(x) > DriveConstants.kDeadband ? x : 0.0;
                    y = Math.abs(y) > DriveConstants.kDeadband ? y : 0.0;
                    rot = Math.abs(rot) > DriveConstants.kDeadband ? rot : 0.0;

                    x *= DriveConstants.kPhysicalMaxSpeed;
                    y *= DriveConstants.kPhysicalMaxSpeed;
                    rot *= DriveConstants.kMaxTeleAngularSpeed;

                    ChassisSpeeds speeds = isRobotRelative.getAsBoolean()
                            ? new ChassisSpeeds(x, y, rot)
                            : ChassisSpeeds.fromFieldRelativeSpeeds(x, y, rot, drive.getRawGyroRotation());

                    drive.runVelocity(speeds, true);
                },
                drive);
    }
}
