package org.team100.lib.commands.drivetrain;

import org.team100.lib.controller.HolonomicDriveController3;
import org.team100.lib.controller.State100;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.logging.SupplierLogger2;
import org.team100.lib.logging.SupplierLogger2.DoubleSupplierLogger2;
import org.team100.lib.logging.SupplierLogger2.State100Logger;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.SwerveState;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.profile.TrapezoidProfile100;
import org.team100.lib.sensors.Gyro;
import org.team100.lib.telemetry.Telemetry.Level;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Rotate in place to the specified angle.
 * 
 * Uses a profile with the holonomic drive controller.
 */
public class Rotate extends Command implements Glassy  {

    private static final double kXToleranceRad = 0.02;
    private static final double kVToleranceRad_S = 0.02;
    // don't try to rotate at max speed
    private static final double kSpeed = 0.5;

    private final SwerveDriveSubsystem m_robotDrive;
    private final Gyro m_gyro;
    private final SwerveKinodynamics m_swerveKinodynamics;
    private final State100 m_goalState;
    // LOGGERS
    private final DoubleSupplierLogger2 m_log_error_x;
    private final DoubleSupplierLogger2 m_log_error_v;
    private final DoubleSupplierLogger2 m_log_measurement_x;
    private final DoubleSupplierLogger2 m_log_measurement_v;
    private final State100Logger m_log_reference;

    final HolonomicDriveController3 m_controller;

    private boolean m_finished = false;

    TrapezoidProfile100 m_profile;
    State100 refTheta;

    private boolean m_steeringAligned;

    public Rotate(
            SupplierLogger2 parent,
            SwerveDriveSubsystem drivetrain,
            Gyro gyro,
            SwerveKinodynamics swerveKinodynamics,
            double targetAngleRadians) {
        SupplierLogger2 child = parent.child(this);
        m_robotDrive = drivetrain;
        // since we specify a different tolerance, use a new controller.

        PIDController xc = HolonomicDriveController3.cartesian();
        xc.setTolerance(0.1, 0.1);
        PIDController yc = HolonomicDriveController3.cartesian();
        yc.setTolerance(0.1, 0.1);
        PIDController tc = HolonomicDriveController3.theta();
        tc.setTolerance(kXToleranceRad, kVToleranceRad_S);
        // in testing, the default theta p causes overshoot, but i think this isn't a
        // real effect.
        tc.setP(3.5);

        m_controller = new HolonomicDriveController3(child, xc, yc, tc);
        m_gyro = gyro;
        m_swerveKinodynamics = swerveKinodynamics;
        m_goalState = new State100(targetAngleRadians, 0);
        refTheta = new State100(0, 0);

        addRequirements(drivetrain);
        m_log_error_x = child.doubleLogger(Level.TRACE, "errorX");
        m_log_error_v = child.doubleLogger(Level.TRACE, "errorV");
        m_log_measurement_x = child.doubleLogger(Level.TRACE, "measurementX");
        m_log_measurement_v = child.doubleLogger(Level.TRACE, "measurementV");
        m_log_reference = child.state100Logger(Level.TRACE, "reference");
    }

    @Override
    public void initialize() {
        m_controller.reset();
        resetRefTheta(0.02);
        m_profile = new TrapezoidProfile100(
                m_swerveKinodynamics.getMaxAngleSpeedRad_S() * kSpeed,
                m_swerveKinodynamics.getMaxAngleAccelRad_S2() * kSpeed,
                0.05);
        // first align the wheels
        m_steeringAligned = false;
    }

    private void resetRefTheta(double dt) {
        ChassisSpeeds initialSpeeds = m_robotDrive.getState().chassisSpeeds();
        refTheta = new State100(
                m_robotDrive.getState().pose().getRotation().getRadians(),
                initialSpeeds.omegaRadiansPerSecond);
    }

    @Override
    public void execute() {
        double dt = 0.02;
        // reference
        refTheta = m_profile.calculate(dt, refTheta, m_goalState);
        m_finished = MathUtil.isNear(refTheta.x(), m_goalState.x(), kXToleranceRad)
                && MathUtil.isNear(refTheta.v(), m_goalState.v(), kVToleranceRad_S);

        // measurement
        Pose2d currentPose = m_robotDrive.getState().pose();

        SwerveState reference = new SwerveState(
                new State100(currentPose.getX(), 0, 0), // stationary at current pose
                new State100(currentPose.getY(), 0, 0),
                new State100(refTheta.x(), refTheta.v(), refTheta.a()));

        FieldRelativeVelocity fieldRelativeTarget = m_controller.calculate(currentPose, reference);

        if (m_steeringAligned) {
            // steer normally.
            // there's no feasibility issue because cartesian speed is zero.
            m_robotDrive.driveInFieldCoords(fieldRelativeTarget, dt);
        } else {
            boolean aligned = m_robotDrive.steerAtRest(fieldRelativeTarget, dt);
            // while waiting for the wheels, hold the profile at the start.
            resetRefTheta(dt);
            if (aligned) {
                m_steeringAligned = true;
            }
        }

        double headingMeasurement = currentPose.getRotation().getRadians();
        double headingRate = m_gyro.getYawRateNWU();

        // log what we did
        m_log_error_x.log(() -> refTheta.x() - headingMeasurement);
        m_log_error_v.log(() -> refTheta.v() - headingRate);
        m_log_measurement_x.log(() -> headingMeasurement);
        m_log_measurement_v.log(() -> headingRate);
        m_log_reference.log(() -> refTheta);
    }

    @Override
    public boolean isFinished() {
        return m_finished && m_controller.atReference();
    }

    @Override
    public void end(boolean isInterupted) {
        m_robotDrive.stop();
    }
}
