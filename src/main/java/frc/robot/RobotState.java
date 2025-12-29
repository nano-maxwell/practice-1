package frc.robot;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.drive.DriveConstants;
import lombok.Getter;
import lombok.Setter;

public class RobotState {
    private static Rotation2d robotHeading = new Rotation2d();
    private static SwerveModulePosition[] modulePositions = new SwerveModulePosition[] {
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
    };

    private static final SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(
            DriveConstants.swerveKinematics, new Rotation2d(), modulePositions, new Pose2d(8.27, 4.01, Rotation2d.kZero));
    // private static final SwerveDriveOdometry m_odometry = new
    // SwerveDriveOdometry(DriveConstants.swerveKinematics,
    // new Rotation2d(), modulePositions);

    @Getter
    @AutoLogOutput(key = "RobotState/EstimatedPose")
    private Pose2d m_estimatedPose = Pose2d.kZero;

    @Getter
    @Setter
    private static RobotMode mode;

    @Getter
    public RobotState instance = new RobotState();

    public RobotState() {
    }

    public static void periodic(
            Rotation2d robotHeading,
            SwerveModulePosition[] modulePositions) {
        RobotState.robotHeading = robotHeading;
        RobotState.modulePositions = modulePositions;

        m_poseEstimator.updateWithTime(Timer.getTimestamp(), robotHeading, modulePositions);

        Logger.recordOutput("RobotState/Pose", m_poseEstimator.getEstimatedPosition());
    }

    public void resetPose(Pose2d pose) {
        // TODO: Finish method
    }

    public enum RobotMode {
        DISABLED,
        TELEOP,
        AUTO;

        public static boolean enabled(RobotMode mode) {
            return mode.equals(TELEOP) || mode.equals(AUTO);
        }

        public static boolean disabled(RobotMode mode) {
            return mode.equals(DISABLED);
        }

        public static boolean teleop(RobotMode mode) {
            return mode.equals(TELEOP);
        }

        public static boolean auto(RobotMode mode) {
            return mode.equals(AUTO);
        }

        public static boolean enabled() {
            return enabled(RobotState.getMode());
        }

        public static boolean disabled() {
            return disabled(RobotState.getMode());
        }

        public static boolean teleop() {
            return teleop(RobotState.getMode());
        }

        public static boolean auto() {
            return auto(RobotState.getMode());
        }
    }
}
