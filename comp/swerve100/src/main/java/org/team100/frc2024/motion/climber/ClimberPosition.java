package org.team100.frc2024.motion.climber;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.motion.components.LinearPositionServo;
import org.team100.lib.motion.components.OnboardLinearDutyCyclePositionServo;
import org.team100.lib.profile.TrapezoidProfile100;
import org.team100.lib.telemetry.SupplierLogger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

public class ClimberPosition extends Command implements Glassy {
    private static final double kToleranceM = 0.01;

    private final LinearPositionServo m_leftServo;
    private final LinearPositionServo m_rightServo;
    private final SupplierLogger m_logger;
    private final ClimberSubsystem m_climber;
    private final double m_goalM;

    public ClimberPosition(
            SupplierLogger logger,
            double goalM,
            ClimberSubsystem climber) {
        m_logger = logger.child(this);
        m_goalM = goalM;
        m_climber = climber;
        m_leftServo = new OnboardLinearDutyCyclePositionServo(
                m_logger,
                m_climber.getLeft(),
                new PIDController(0.1, 0, 0),
                0.02,
                new TrapezoidProfile100(0.02, 0.1, 0.01));
        m_rightServo = new OnboardLinearDutyCyclePositionServo(
                m_logger,
                m_climber.getRight(),
                new PIDController(0.1, 0, 0),
                0.02,
                new TrapezoidProfile100(0.02, 0.1, 0.02));
    }

    @Override
    public void initialize() {
        m_leftServo.reset();
        m_rightServo.reset();
        m_climber.setClimbingForce();
    }

    @Override
    public void execute() {
        m_leftServo.setPosition(m_goalM);
        m_rightServo.setPosition(m_goalM);
    }

    @Override
    public boolean isFinished() {
        return MathUtil.isNear(
                m_goalM,
                m_climber.getLeft().getPositionM().orElse(m_goalM),
                kToleranceM) &&
                MathUtil.isNear(
                        m_goalM,
                        m_climber.getRight().getPositionM().orElse(m_goalM),
                        kToleranceM);
    }

    @Override
    public void end(boolean interrupted) {
        m_leftServo.stop();
        m_rightServo.stop();
    }

    @Override
    public String getGlassName() {
        return "ClimberPosition";
    }
}
