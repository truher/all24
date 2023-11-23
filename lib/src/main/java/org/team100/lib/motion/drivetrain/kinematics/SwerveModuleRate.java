package org.team100.lib.motion.drivetrain.kinematics;

import java.util.Objects;

/**
 * Cribbed from 829, https://github.com/narmstro2020/SecondOrderExperiments/
 * 
 * //Math based on paper located at
 * https://www.chiefdelphi.com/uploads/short-url/qzj4k2LyBs7rLxAem0YajNIlStH.pdf
 * 
 * Also see Chief Delphi thread at
 * https://www.chiefdelphi.com/t/whitepaper-swerve-drive-skew-and-second-order-kinematics/416964
 */

public class SwerveModuleRate implements Comparable<SwerveModuleRate> {
    public double accelerationMetersPerSecondSquared;

    public double angularVelocityRadiansPerSecond;

    public SwerveModuleRate() {
    }

    public SwerveModuleRate(double accelerationMetersPerSecondSquared, double angularVelocityRadiansPerSecond) {
        this.accelerationMetersPerSecondSquared = accelerationMetersPerSecondSquared;
        this.angularVelocityRadiansPerSecond = angularVelocityRadiansPerSecond;
    }

    @Override
    public boolean equals(Object obj) {
        if (obj instanceof SwerveModuleRate) {
            SwerveModuleRate other = (SwerveModuleRate) obj;
            return Math.abs(other.accelerationMetersPerSecondSquared - accelerationMetersPerSecondSquared) < 1E-9
                    && angularVelocityRadiansPerSecond == other.angularVelocityRadiansPerSecond;
        }
        return false;
    }

    @Override
    public int hashCode() {
        return Objects.hash(accelerationMetersPerSecondSquared, angularVelocityRadiansPerSecond);
    }

    @Override
    public int compareTo(SwerveModuleRate other) {
        return Double.compare(this.accelerationMetersPerSecondSquared, other.accelerationMetersPerSecondSquared);
    }

    @Override
    public String toString() {
        return String.format(
                "SwerveModuleRate(Acceleration: %.2f m/s^2, AngularVelocity: %s rad/s)",
                accelerationMetersPerSecondSquared,
                angularVelocityRadiansPerSecond);
    }
}