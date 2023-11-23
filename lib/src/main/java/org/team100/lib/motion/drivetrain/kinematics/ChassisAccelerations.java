package org.team100.lib.motion.drivetrain.kinematics;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Cribbed from 829, https://github.com/narmstro2020/SecondOrderExperiments/
 * 
 * Math based on paper located at
 * https://www.chiefdelphi.com/uploads/short-url/qzj4k2LyBs7rLxAem0YajNIlStH.pdf
 * 
 * Also see Chief Delphi thread at
 * https://www.chiefdelphi.com/t/whitepaper-swerve-drive-skew-and-second-order-kinematics/416964
 */
public class ChassisAccelerations {
    public double axMetersPerSecondSquared;
    public double ayMetersPerSecondSquared;
    public double centripetalAccelerationRadSquaredPerSecSquared;
    public double alphaRadiansPerSecondSquared;

    public ChassisAccelerations() {
    }

    public ChassisAccelerations(
            ChassisSpeeds currentChassisSpeeds,
            ChassisSpeeds nextChassisSpeeds) {
        this(
                (nextChassisSpeeds.vxMetersPerSecond - currentChassisSpeeds.vxMetersPerSecond) / 0.020,
                (nextChassisSpeeds.vyMetersPerSecond - currentChassisSpeeds.vyMetersPerSecond) / 0.020,
                currentChassisSpeeds.omegaRadiansPerSecond * currentChassisSpeeds.omegaRadiansPerSecond,
                (nextChassisSpeeds.omegaRadiansPerSecond - currentChassisSpeeds.omegaRadiansPerSecond) / 0.020);
    }

    public ChassisAccelerations(
            double axMetersPerSecondSquared,
            double ayMetersPerSecondSquared,
            double centripetalAccelerationRadSquaredPerSecSquared,
            double alphaRadiansPerSecondSquared) {
        this.axMetersPerSecondSquared = axMetersPerSecondSquared;
        this.ayMetersPerSecondSquared = ayMetersPerSecondSquared;
        this.centripetalAccelerationRadSquaredPerSecSquared = centripetalAccelerationRadSquaredPerSecSquared;
        this.alphaRadiansPerSecondSquared = alphaRadiansPerSecondSquared;
    }

    public static ChassisAccelerations fromFieldRelativeAccelerations(
            double axMetersPerSecondSquared,
            double ayMetersPerSecondSquared,
            double centripetalAccelerationRadSquaredPerSecSquared,
            double alphaRadiansPerSecondSquared,
            Rotation2d robotAngle) {
        return new ChassisAccelerations(
                axMetersPerSecondSquared * robotAngle.getCos() + ayMetersPerSecondSquared * robotAngle.getSin(),
                -axMetersPerSecondSquared * robotAngle.getSin() + ayMetersPerSecondSquared * robotAngle.getCos(),
                centripetalAccelerationRadSquaredPerSecSquared,
                alphaRadiansPerSecondSquared);
    }

    public static ChassisAccelerations fromFieldRelativeAccelerations(
            ChassisAccelerations fieldRelativeAccelerations, Rotation2d robotAngle) {
        return fromFieldRelativeAccelerations(
                fieldRelativeAccelerations.axMetersPerSecondSquared,
                fieldRelativeAccelerations.ayMetersPerSecondSquared,
                fieldRelativeAccelerations.centripetalAccelerationRadSquaredPerSecSquared,
                fieldRelativeAccelerations.alphaRadiansPerSecondSquared,
                robotAngle);
    }

    @Override
    public String toString() {
        return String.format(
                "ChassisAccelerations(Ax: %.2f m/s^2, Ay: %.2f m/s^2, Centrip: %.2f rad^2/s^2, Alpha: %.2f rad/s^2)",
                axMetersPerSecondSquared, ayMetersPerSecondSquared, centripetalAccelerationRadSquaredPerSecSquared,
                alphaRadiansPerSecondSquared);
    }
}