package frc.robot.subsystems.drive.module;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.drive.DriveConstants;

public class ModuleIOSim implements ModuleIO {
    private DCMotor driveMotorModel = DCMotor.getKrakenX60Foc(1);
    private DCMotor turnMotorModel = DCMotor.getNEO(1);

    private DCMotorSim driveMotorSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(driveMotorModel, 0.025, DriveConstants.kDriveGearRatio),
            driveMotorModel);
    private DCMotorSim turnMotorSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(turnMotorModel, 0.004, DriveConstants.kAngleGearRatio), turnMotorModel);

    private boolean driveClosedLoop = false;
    private boolean turnClosedLoop = false;
    private PIDController driveController = new PIDController(2, 0, 0);
    private PIDController turnController = new PIDController(5, 0, 0);
    private double driveFFVolts = 0;
    private double driveAppliedVolts = 0.0;
    private double turnAppliedVolts = 0.0;

    public ModuleIOSim() {
        turnController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        if (driveClosedLoop) {
            driveAppliedVolts = driveFFVolts + driveController.calculate(driveMotorSim.getAngularVelocityRadPerSec());
        } else {
            driveController.reset();
        }
        if (turnClosedLoop) {
            turnAppliedVolts = turnController.calculate(turnMotorSim.getAngularPositionRad());
        } else {
            turnController.reset();
        }

        driveMotorSim.setInputVoltage(MathUtil.clamp(driveAppliedVolts, -12.0, 12.0));
        turnMotorSim.setInputVoltage(MathUtil.clamp(turnAppliedVolts, -12.0, 12.0));
        driveMotorSim.update(0.02);
        turnMotorSim.update(0.02);

        inputs.data = new ModuleIOData(
                true,
                driveMotorSim.getAngularPositionRad(),
                driveMotorSim.getAngularVelocityRadPerSec(),
                driveAppliedVolts,
                true,
                Rotation2d.fromRadians(turnMotorSim.getAngularPositionRad()),
                turnMotorSim.getAngularVelocityRadPerSec(),
                turnAppliedVolts,
                driveMotorSim.getCurrentDrawAmps(),
                turnMotorSim.getCurrentDrawAmps());
    }

    @Override
    public void runDriveDutyCycle(double percentOutput) {
        driveClosedLoop = false;
        driveAppliedVolts = percentOutput * 12;
    }

    @Override
    public void runTurnDutyCycle(double percentOutput) {
        turnClosedLoop = false;
        turnAppliedVolts = percentOutput * 12;
    }

    @Override
    public void runDriveVelocity(double velocityRadPerSec, double feedforward) {
        driveClosedLoop = true;
        driveFFVolts = feedforward;
        driveController.setSetpoint(velocityRadPerSec);
    }

    @Override
    public void runTurnAngle(Rotation2d angle) {
        turnClosedLoop = true;
        turnController.setSetpoint(angle.getRadians());
    }
}
