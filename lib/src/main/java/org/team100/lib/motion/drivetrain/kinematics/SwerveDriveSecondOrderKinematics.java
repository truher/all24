package org.team100.lib.motion.drivetrain.kinematics;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import java.util.Arrays;
import org.ejml.simple.SimpleMatrix;

/**
 * Cribbed from 829, https://github.com/narmstro2020/SecondOrderExperiments/
 * 
 * //Math based on paper located at
 * https://www.chiefdelphi.com/uploads/short-url/qzj4k2LyBs7rLxAem0YajNIlStH.pdf
 * 
 * Also see Chief Delphi thread at
 * https://www.chiefdelphi.com/t/whitepaper-swerve-drive-skew-and-second-order-kinematics/416964
 */
public class SwerveDriveSecondOrderKinematics {
    private final SimpleMatrix m_inverseKinematics;
    private final SimpleMatrix m_forwardKinematics;

    private final int m_numModules;
    private final Translation2d[] m_modules;
    private SwerveModuleRate[] m_moduleRates;

    public SwerveDriveSecondOrderKinematics(Translation2d... wheelsMeters) {
        if (wheelsMeters.length < 2) {
            throw new IllegalArgumentException("A swerve drive requires at least two modules");
        }
        m_numModules = wheelsMeters.length;
        m_modules = Arrays.copyOf(wheelsMeters, m_numModules);
        m_moduleRates = new SwerveModuleRate[m_numModules];
        Arrays.fill(m_moduleRates, new SwerveModuleRate());
        m_inverseKinematics = new SimpleMatrix(m_numModules * 2, 4);

        for (int i = 0; i < m_numModules; i++) {
            m_inverseKinematics.setRow(i * 2 + 0, 0, /* Start Data */ 1, 0, -m_modules[i].getX(), -m_modules[i].getY());
            m_inverseKinematics.setRow(i * 2 + 1, 0, /* Start Data */ 0, 1, -m_modules[i].getY(), +m_modules[i].getX());
        }
        m_forwardKinematics = m_inverseKinematics.pseudoInverse();
    }

    public SwerveModuleRate[] toSwerveModuleRates(
            ChassisAccelerations chassisAccelerations,
            SwerveModuleState[] swerveModuleStates) {
        if (chassisAccelerations.axMetersPerSecondSquared == 0.0
                && chassisAccelerations.ayMetersPerSecondSquared == 0.0
                && chassisAccelerations.centripetalAccelerationRadSquaredPerSecSquared == 0.0
                && chassisAccelerations.alphaRadiansPerSecondSquared == 0.0) {
            SwerveModuleRate[] newRates = new SwerveModuleRate[m_numModules];
            for (int i = 0; i < m_numModules; i++) {
                newRates[i] = new SwerveModuleRate(0.0, 0.0);
            }

            m_moduleRates = newRates;
            return m_moduleRates;
        }

        var chassisAccelerationsVector = new SimpleMatrix(4, 1);
        chassisAccelerationsVector.setColumn(
                0,
                0,
                chassisAccelerations.axMetersPerSecondSquared,
                chassisAccelerations.ayMetersPerSecondSquared,
                chassisAccelerations.centripetalAccelerationRadSquaredPerSecSquared,
                chassisAccelerations.alphaRadiansPerSecondSquared);

        var moduleRatesMatrix = m_inverseKinematics.mult(chassisAccelerationsVector);

        m_moduleRates = new SwerveModuleRate[m_numModules];
        for (int i = 0; i < m_numModules; i++) {
            double x = moduleRatesMatrix.get(i * 2, 0);
            double y = moduleRatesMatrix.get(i * 2 + 1, 0);

            Rotation2d moduleAngle = swerveModuleStates[i].angle;
            double moduleMetersPerSecond = swerveModuleStates[i].speedMetersPerSecond;

            double acceleration = x * moduleAngle.getCos() + y * moduleAngle.getSin();
            double angularVelocity = (-x * moduleAngle.getSin() / moduleMetersPerSecond)
                    + (y * moduleAngle.getCos() / moduleMetersPerSecond);

            m_moduleRates[i] = new SwerveModuleRate(acceleration, angularVelocity);
        }

        return m_moduleRates;
    }

    public ChassisAccelerations toChassisAccelerations(SwerveModuleState[] wheelStates, SwerveModuleRate[] wheelRates) {
        if (wheelRates.length != m_numModules) {
            throw new IllegalArgumentException(
                    "Number of modules is not consistent with number of wheel locations provided in "
                            + "constructor");
        }
        var moduleRatesMatrix = new SimpleMatrix(m_numModules * 2, 1);

        for (int i = 0; i < m_numModules; i++) {
            var module = wheelRates[i];
            var moduleState = wheelStates[i];
            moduleRatesMatrix.set(i * 2, 0, module.accelerationMetersPerSecondSquared * moduleState.angle.getCos());
            moduleRatesMatrix.set(i * 2 + 1, module.accelerationMetersPerSecondSquared * moduleState.angle.getSin());
        }

        var chassisAccelerationVector = m_forwardKinematics.mult(moduleRatesMatrix);
        return new ChassisAccelerations(
                chassisAccelerationVector.get(0, 0),
                chassisAccelerationVector.get(1, 0),
                chassisAccelerationVector.get(2, 0),
                chassisAccelerationVector.get(3, 0));
    }

}