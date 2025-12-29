package frc.robot.subsystems.drive.module;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.subsystems.drive.DriveConstants;
import lombok.Getter;

/**
 * The {@code SwerveMod} class contains/controls the io, inputs, and name of
 * one swerve module.
 */
public class SwerveMod {
    private final ModuleIO m_io;

    @Getter
    private final ModuleIOInputsAutoLogged m_inputs = new ModuleIOInputsAutoLogged();
    @Getter
    private final ModuleName m_name;

    private SimpleMotorFeedforward m_ffController;

    public SwerveMod(ModuleIO io, ModuleName name) {
        this.m_io = io;
        this.m_name = name;

        m_ffController = new SimpleMotorFeedforward(
                DriveConstants.kDriveKS, DriveConstants.kDriveKV, DriveConstants.kDriveKA);
    }

    public void periodic() {
        m_io.updateInputs(m_inputs);
        Logger.processInputs("Drive/Module/" + m_name.toString(), m_inputs);
    }

    /**
     * Runs the module to the desired state.
     * 
     * @param desiredState Desired wheel speed and angle
     * @param isOpenLoop   If true, uses percent output; if false, uses velocity
     *                     closed-loop
     */
    public void runDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        // Cosine compensation reduces speed if wheel isn't pointing correctly
        double angleDiff = desiredState.angle.minus(getAngle()).getRadians();
        double compensatedSpeed = desiredState.speedMetersPerSecond * Math.cos(angleDiff);
        double speedRadPerSec = compensatedSpeed / DriveConstants.kWheelRadius;

        if (isOpenLoop) {
            double percentOutput = compensatedSpeed / DriveConstants.kPhysicalMaxSpeed;
            m_io.runDriveDutyCycle(percentOutput);
        } else {
            double ff = m_ffController.calculate(speedRadPerSec);
            m_io.runDriveVelocity(speedRadPerSec, ff);
        }

        // Turn control
        if (isTurnWithinDeadband(desiredState.angle)) {
            m_io.runTurnDutyCycle(0);
        } else {
            m_io.runTurnAngle(desiredState.angle);
        }
    }

    /** Stops all output to the module's motors. */
    public void stop() {
        m_io.runDriveDutyCycle(0);
        m_io.runTurnDutyCycle(0);
    }

    public void resetToAbsolute() {
        m_io.resetToAbsolute();
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocityMetersPerSec(), m_inputs.data.turnPosition());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getPositionMeters(), m_inputs.data.turnPosition());
    }

    public Rotation2d getAngle() {
        return m_inputs.data.turnPosition();
    }

    public double getPositionMeters() {
        return m_inputs.data.drivePositionRad() * DriveConstants.kWheelRadius;
    }

    public double getVelocityMetersPerSec() {
        return m_inputs.data.driveVelocityRadPerSec() * DriveConstants.kWheelRadius;
    }

    private boolean isTurnWithinDeadband(Rotation2d target) {
        return Math.abs(target.minus(getAngle()).getDegrees()) < 3;
    }

    public enum ModuleName {
        FRONT_LEFT(0),
        FRONT_RIGHT(1),
        BACK_LEFT(2),
        BACK_RIGHT(3);

        @Getter
        private final int index;

        ModuleName(int index) {
            this.index = index;
        }
    }
}
