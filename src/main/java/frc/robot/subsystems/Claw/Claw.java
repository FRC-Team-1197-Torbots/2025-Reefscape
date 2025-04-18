package frc.robot.subsystems.Claw;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.TimingBudget;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Configs.ClawConfig;
import frc.robot.subsystems.TorSubsystemBase;
import frc.robot.subsystems.Claw.ClawConstants.CoralIntakeConstants;
import frc.robot.subsystems.Claw.ClawConstants.WristConstants;
import frc.robot.subsystems.Drive.DriveAutomation.AligningConstants;

public class Claw extends TorSubsystemBase {
    
    private final SparkFlex m_wristMotor;
    private final SparkFlex m_intakeMotor;
    private final SparkClosedLoopController m_wristController;
    private final SparkClosedLoopController m_intakeController;
    private final RelativeEncoder m_encoder; 
    private double targetAngle;
    private double currentSetpoint;
    private final LaserCan m_backLaser;
    private final LaserCan m_frontLaser;
    public boolean positioningCoral;

    public Claw() {
        m_wristMotor = new SparkFlex(WristConstants.MotarCanId, MotorType.kBrushless);
        m_wristMotor.configure(ClawConfig.wristMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_wristController = m_wristMotor.getClosedLoopController();
        m_encoder = m_wristMotor.getEncoder();
        m_encoder.setPosition(WristConstants.Initial);

        m_intakeMotor = new SparkFlex(CoralIntakeConstants.MotorCanId, MotorType.kBrushless);
        m_intakeMotor.configure(ClawConfig.intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_intakeController = m_intakeMotor.getClosedLoopController();
        resetSetpoint();

        m_backLaser = new LaserCan(AligningConstants.BackCoralLaserCanID);
        m_frontLaser = new LaserCan(AligningConstants.FrontCoralLaserCanID);
        try {
            m_backLaser.setTimingBudget(TimingBudget.TIMING_BUDGET_20MS);
        } catch (Exception e) {
            System.out.println("laser can is the worst");
        }
    }

    public void resetSetpoint() {
        currentSetpoint = getAngle();
        // targetAngle = currentSetpoint;
    }

    public double getAngle() {
        return m_encoder.getPosition();
    }

    public void runIntake() {
        runVoltage(CoralIntakeConstants.IntakeVoltage);
    }

    public void runOuttake() {
        runVoltage(CoralIntakeConstants.OuttakeVoltage);
    }

    public void runVoltage(double volts) {
        m_intakeController.setReference(volts, ControlType.kVoltage);
    }

    @Override
    public void setZero() {
        m_encoder.setPosition(0);
    }

    public void runClaw (double volts) {
        m_wristController.setReference(volts, ControlType.kVoltage);
    }

    public void stopIntake() {
        m_intakeMotor.stopMotor();
    }

    public void setTargetAngle(double angle) {
        if(angle != targetAngle) {
            targetAngle = MathUtil.clamp(angle, WristConstants.MinAngle, WristConstants.MaxAngle); 
            currentSetpoint = getAngle();
        }
    }

    private void positionSlewRateLimiting() {
        double error = targetAngle - currentSetpoint;
        currentSetpoint += Math.min(Math.abs(error), WristConstants.SlewRate) * Math.signum(error);
        m_wristController.setReference(currentSetpoint, ControlType.kPosition);
    }

    public boolean onTarget() {
        return Math.abs(m_encoder.getPosition() - targetAngle) < WristConstants.Tolerance;
    }

    public double getDistance() {
        if (m_backLaser.getMeasurement() == null)
            return 0;
        else
            return m_backLaser.getMeasurement().distance_mm;
    }

    public boolean backLaserTriggered() {
        return (m_backLaser.getMeasurement() != null) && (m_backLaser.getMeasurement().status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) && getDistance() < 100;
    }

    public boolean frontLaserTriggered() {
        return (m_frontLaser.getMeasurement() != null) && (m_frontLaser.getMeasurement().status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) && getDistance() < 100;
    }
                                    
    @Override
    public void periodic() {
        positionSlewRateLimiting();
        SmartDashboard.putNumber("Wrist Angle", getAngle());
        SmartDashboard.putNumber("Intake Velocity", m_intakeMotor.getEncoder().getVelocity());
        SmartDashboard.putBoolean("Front Coral Stored", frontLaserTriggered());
        SmartDashboard.putBoolean("Back Coral Stored", backLaserTriggered());
        SmartDashboard.putNumber("BackLaserCan Distance", getDistance());
        SmartDashboard.putBoolean("Claw on target", onTarget());

    }

    @Override
    public void toggleIdleMode() {
        SparkBaseConfig config = super.getIdleModeConfig();
        m_wristMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public boolean isBrakeMode() {
        return m_wristMotor.configAccessor.getIdleMode() == IdleMode.kBrake;
    }
}
