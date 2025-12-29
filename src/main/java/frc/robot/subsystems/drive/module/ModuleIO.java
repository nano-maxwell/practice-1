package frc.robot.subsystems.drive.module;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface ModuleIO {
    default void updateInputs(ModuleIOInputs inputs) {
    }

    @AutoLog
    /** Module values */
    public class ModuleIOInputs {
        public ModuleIOData data = new ModuleIOData(false, 0, 0, 0, false, Rotation2d.kZero, 0, 0, 0, 0);
    }

    public record ModuleIOData(
            boolean driveConnected,
            double drivePositionRad,
            double driveVelocityRadPerSec,
            double driveAppliedVolts,
            boolean turnConnected,
            Rotation2d turnPosition,
            double turnVelocityRadPerSec,
            double turnAppliedVolts,
            double driveCurrentAmps,
            double turnCurrentAmps) {
    }

    /** Sets the drive motor speed (output % as decimal). */
    default void runDriveDutyCycle(double percentOutput) {
    }

    /** Sets the turn motor speed (output % as decimal). */
    default void runTurnDutyCycle(double percentOutput) {
    }

    /** Sets the drive motor velocity. */
    default void runDriveVelocity(double velocityRadPerSec, double feedforward) {
    }

    /** Sets the turn motor to the specified angle. */
    default void runTurnAngle(Rotation2d angle) {
    }

    /** Resets the turning encoder to match absolute CANcoder. */
    default void resetToAbsolute() {
    }
}
