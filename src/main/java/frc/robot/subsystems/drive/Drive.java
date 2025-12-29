package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.GyroIO.GyroIOData;
import frc.robot.subsystems.drive.module.SwerveMod;
import lombok.Getter;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;

public class Drive extends SubsystemBase {
    private final SwerveMod[] m_modules;
    private final GyroIO m_gyroIO;
    private final GyroIOInputsAutoLogged m_gyroIOInputs = new GyroIOInputsAutoLogged();

    private Rotation2d m_rawGyroRotation = new Rotation2d();
    private SwerveModulePosition[] lastModulePositions = new SwerveModulePosition[] {
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
    };

    static final Lock m_odometryLock = new ReentrantLock();

    public Drive(SwerveMod[] modules, GyroIO gyroIO) {
        m_modules = modules;
        this.m_gyroIO = gyroIO;
    }

    @Override
    public void periodic() {
        m_odometryLock.lock(); // Prevents odometry updates while reading data
        m_gyroIO.updateInputs(m_gyroIOInputs);
        Logger.processInputs("Drive/Gyro", m_gyroIOInputs);
        for (SwerveMod mod : m_modules) {
            mod.periodic();
        }
        m_odometryLock.unlock();

        if (DriverStation.isDisabled()) {
            for (SwerveMod mod : m_modules) {
                mod.stop();
            }
        }

        SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];

        for (int i = 0; i < 4; i++) {
            SwerveModulePosition current = m_modules[i].getPosition();
            moduleDeltas[i] = new SwerveModulePosition(
                    current.distanceMeters - lastModulePositions[i].distanceMeters,
                    current.angle);
            lastModulePositions[i] = current;
        }

        if (m_gyroIOInputs.data.connected()) {
            m_rawGyroRotation = m_gyroIOInputs.data.yawPosition();
        } else {
            Twist2d twist = DriveConstants.swerveKinematics.toTwist2d(moduleDeltas);
            m_rawGyroRotation = m_rawGyroRotation.plus(new Rotation2d(twist.dtheta));
        }
    }

    public void runVelocity(ChassisSpeeds speeds, boolean isOpenLoop) {
        var states = DriveConstants.swerveKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(
                states, DriveConstants.kMaxTeleDriveSpeed);
        for (int i = 0; i < 4; i++) {
            states[i].optimize(m_modules[i].getAngle());
            m_modules[i].runDesiredState(states[i], isOpenLoop);
        }
    }

    public void stopModules() {
        for (SwerveMod mod : m_modules) {
            mod.stop();
        }
    }

    public Rotation2d getRawGyroRotation() {
        return this.m_rawGyroRotation;
    }

    public double getYawVelocity() {
        return this.m_gyroIOInputs.data.yawVelocityRadPerSec();
    }

    /**
     * Retrieves the current position of all swerve modules.
     * 
     * @return An array of {@link SwerveModulePosition} objects, one for each sweve
     *         module, ordered according to the module array in {@code m_modules}.
     */
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];

        for (int i = 0; i < m_modules.length; i++) {
            modulePositions[i] = m_modules[i].getPosition();
        }

        return modulePositions;
    }

    /**
     * Retrieves the current state of all swerve modules.
     * 
     * @return An array of {@link SwerveModuleState} objects, one for each sweve
     *         module, ordered according to the module array in {@code m_modules}.
     */
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] moduleStates = new SwerveModuleState[4];

        for (int i = 0; i < m_modules.length; i++) {
            moduleStates[i] = m_modules[i].getState();
        }

        return moduleStates;
    }
}