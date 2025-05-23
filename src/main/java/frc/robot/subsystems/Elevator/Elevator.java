package frc.robot.subsystems.Elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.Level;
import frc.robot.subsystems.TorSubsystemBase;
import frc.robot.subsystems.Configs.ElevatorConfig;

public class Elevator extends TorSubsystemBase {
    
    private final SparkFlex m_leftMotor;
    private final SparkFlex m_rightMotor; // follower motor
    private final SparkClosedLoopController m_controller;
    private final RelativeEncoder m_encoder;
    private final TrapezoidProfile m_TrapezoidProfile = new TrapezoidProfile(new Constraints(ElevatorConstants.MaxVelocity, ElevatorConstants.MaxAcceleration));
    private State targetState = new State(0, 0);
    private State currentState = new State(0, 0);

    public Elevator() {
        m_leftMotor = new SparkFlex(ElevatorConstants.leftMotorId, MotorType.kBrushless);
        m_leftMotor.configure(ElevatorConfig.leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_rightMotor = new SparkFlex(ElevatorConstants.rightMotorId, MotorType.kBrushless);
        m_rightMotor.configure(ElevatorConfig.rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_controller = m_leftMotor.getClosedLoopController();
        m_encoder = m_leftMotor.getEncoder();
        m_encoder.setPosition(0);
        resetSetpoint();
    }

    public void resetSetpoint() {
        currentState = new State(getHeight(), 0);
        // targetState = currentState;
    }

    
    public void setTarget(double height) {
        double targetPosition = MathUtil.clamp(height, ElevatorConstants.MinHeight, ElevatorConstants.MaxHeight); 
        if(targetPosition != targetState.position) {
            targetState = new State(targetPosition, 0);
            currentState = new State(getHeight(),m_encoder.getVelocity());
        }
    }

    
    public double getHeight() {
        return m_encoder.getPosition();
    }
    
    public boolean onTarget() {
        return Math.abs(currentState.position - targetState.position) < ElevatorConstants.Tolerance;
    }    
    
    public boolean almostAtStore() {
        return Math.abs(m_encoder.getPosition() - targetState.position) < ElevatorConstants.NotTippable;
    }

    public boolean canDriveAwayFromNet() {
        return Math.abs(m_encoder.getPosition() - ElevatorConstants.Store) < 0.5;
    }
    public boolean approachingHeight(Level level) {
        switch (level.withOffset()) {
            case FOUR:
                return Math.abs(m_encoder.getPosition() - targetState.position) < 0.55; //0.55
            case THREE:
                return Math.abs(m_encoder.getPosition() - targetState.position) < 0.4;
            case TWO:
                return Math.abs(m_encoder.getPosition() - targetState.position) < 0.1;
            case TOPALGAEGRAB:
                return Math.abs(m_encoder.getPosition() - targetState.position) < 0.4;
            default:
                return onTarget();
        }
    }

    @Override
    public void periodic() {
        if (!DriverStation.isTestEnabled()) {
            currentState = m_TrapezoidProfile.calculate(0.02, currentState, targetState);
            m_controller.setReference(currentState.position, ControlType.kPosition, ClosedLoopSlot.kSlot0, ElevatorConstants.kG);
        }
        SmartDashboard.putNumber("Left Elevator Encoder", m_encoder.getPosition());
        SmartDashboard.putNumber("Right Elevator Encoder", m_rightMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Elevator vel", m_encoder.getVelocity());
        SmartDashboard.putNumber("Elevator Current", m_leftMotor.getOutputCurrent());
        SmartDashboard.putBoolean("Elevator on target", onTarget());

    }

    public void setZero() {
        m_encoder.setPosition(0);
    }

    public void stopPID() {
        m_controller.setReference(0, ControlType.kVoltage);
    }
    
    public boolean atNetHeight() {
        return m_encoder.getPosition() > 1.05;
    }

    @Override
    public void toggleIdleMode() {
        SparkBaseConfig config = super.getIdleModeConfig();
        m_leftMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        m_rightMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public boolean isBrakeMode() {
        return m_leftMotor.configAccessor.getIdleMode() == IdleMode.kBrake;
    }

}
