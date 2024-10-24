package org.team100.lib.motion.drivetrain.kinodynamics;

import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Optional;

import org.ejml.simple.SimpleMatrix;
import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.util.DriveUtil;
import org.team100.lib.util.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

class SwerveDriveKinematics100Test {
    private static final double kDelta = 0.001;

    @Test
    void testCrab() {
        // in this case the wheels are assumed to turn immediately to the new
        // angle, so this is straight back.
        SwerveDriveKinematics100 kinematics = new SwerveDriveKinematics100(
                new Translation2d(0.5, 0.5), // 1,1
                new Translation2d(0.5, -0.5), // 1,0
                new Translation2d(-0.5, 0.5), // 0,1
                new Translation2d(-0.5, -0.5) // origin
        );
        SwerveModulePosition100[] start = {
                new SwerveModulePosition100(
                        0,
                        Optional.of(Rotation2d.fromRadians(Math.PI / 2))),
                new SwerveModulePosition100(
                        0,
                        Optional.of(Rotation2d.fromRadians(Math.PI / 2))),
                new SwerveModulePosition100(
                        0,
                        Optional.of(Rotation2d.fromRadians(Math.PI / 2))),
                new SwerveModulePosition100(
                        0.0,
                        Optional.of(Rotation2d.fromRadians(Math.PI / 2)))
        };
        SwerveModulePosition100[] end = {
                new SwerveModulePosition100(
                        1,
                        Optional.of(Rotation2d.fromRadians(Math.PI))),
                new SwerveModulePosition100(
                        1,
                        Optional.of(Rotation2d.fromRadians(Math.PI))),
                new SwerveModulePosition100(
                        1,
                        Optional.of(Rotation2d.fromRadians(Math.PI))),
                new SwerveModulePosition100(
                        1,
                        Optional.of(Rotation2d.fromRadians(Math.PI)))
        };

        SwerveModuleDelta[] delta = DriveUtil.modulePositionDelta(start, end);

        assertEquals(1, delta[0].distanceMeters, kDelta);
        assertEquals(Math.PI, delta[0].angle.get().getRadians(), kDelta);

        Twist2d twist = kinematics.toTwist2d(delta);
        assertEquals(-1, twist.dx, kDelta);
        assertEquals(0, twist.dy, kDelta);
        assertEquals(0, twist.dtheta, kDelta);

        // it transforms the starting pose correctly
        Pose2d pStart = new Pose2d(0.5, 0.5, GeometryUtil.kRotationZero);
        Pose2d pEnd = pStart.exp(twist);
        assertEquals(-0.5, pEnd.getX(), kDelta);
        assertEquals(0.5, pEnd.getY(), kDelta);
        assertEquals(0, pEnd.getRotation().getRadians(), kDelta);
    }

    @Test
    void testCrabInverse() {
        // inverse kinematics for the above case
        SwerveDriveKinematics100 m_kinematics = new SwerveDriveKinematics100(
                new Translation2d(0.5, 0.5),
                new Translation2d(0.5, -0.5),
                new Translation2d(-0.5, 0.5),
                new Translation2d(-0.5, -0.5));

        Pose2d pStart = new Pose2d(0.5, 0.5, GeometryUtil.kRotationZero);
        Pose2d pEnd = new Pose2d(-0.5, 1.5, GeometryUtil.kRotationZero);
        Twist2d t = pStart.log(pEnd);
        assertEquals(-1, t.dx, kDelta);
        assertEquals(1, t.dy, kDelta);
        assertEquals(0, t.dtheta, kDelta);

        // the inverse kinematics really just finds the dx and dy for each
        // corner; it doesn't know the path the corners take to get there
        // so it assumes the corner paths are straight lines.
        SwerveModuleDelta[] p = m_kinematics.toSwerveModuleDelta(t);
        assertEquals(Math.sqrt(2), p[0].distanceMeters, kDelta);
        assertEquals(3 * Math.PI / 4, p[0].angle.get().getRadians(), kDelta);
        assertEquals(Math.sqrt(2), p[1].distanceMeters, kDelta);
        assertEquals(3 * Math.PI / 4, p[1].angle.get().getRadians(), kDelta);
        assertEquals(Math.sqrt(2), p[2].distanceMeters, kDelta);
        assertEquals(3 * Math.PI / 4, p[2].angle.get().getRadians(), kDelta);
        assertEquals(Math.sqrt(2), p[3].distanceMeters, kDelta);
        assertEquals(3 * Math.PI / 4, p[3].angle.get().getRadians(), kDelta);
    }

    @Test
    void testRollDelta() {
        // a rotate-and-move case you can do in your head
        // face +x with right rear at origin
        // keep origin corner still, rotate around it
        // in this maneuver the steering doesn't change
        SwerveDriveKinematics100 kinematics = new SwerveDriveKinematics100(
                new Translation2d(0.5, 0.5), // 1,1
                new Translation2d(0.5, -0.5), // 1,0
                new Translation2d(-0.5, 0.5), // 0,1
                new Translation2d(-0.5, -0.5) // origin
        );
        SwerveModulePosition100[] start = {
                new SwerveModulePosition100(
                        0,
                        Optional.of(Rotation2d.fromRadians(3 * Math.PI / 4))),
                new SwerveModulePosition100(
                        0,
                        Optional.of(Rotation2d.fromRadians(Math.PI / 2))),
                new SwerveModulePosition100(
                        0,
                        Optional.of(Rotation2d.fromRadians(Math.PI))),
                new SwerveModulePosition100(
                        0.0,
                        Optional.empty())
        };
        SwerveModulePosition100[] end = {
                new SwerveModulePosition100(
                        Math.sqrt(2) * Math.PI / 2,
                        Optional.of(Rotation2d.fromRadians(3 * Math.PI / 4))),
                new SwerveModulePosition100(
                        Math.PI / 2,
                        Optional.of(Rotation2d.fromRadians(Math.PI / 2))),
                new SwerveModulePosition100(
                        Math.PI / 2,
                        Optional.of(Rotation2d.fromRadians(Math.PI))),
                new SwerveModulePosition100(
                        0,
                        Optional.empty())
        };

        SwerveModuleDelta[] delta = DriveUtil.modulePositionDelta(start, end);
        assertEquals(Math.sqrt(2) * Math.PI / 2, delta[0].distanceMeters, kDelta);
        assertEquals(3 * Math.PI / 4, delta[0].angle.get().getRadians(), kDelta);
        assertEquals(Math.PI / 2, delta[1].distanceMeters, kDelta);
        assertEquals(Math.PI / 2, delta[1].angle.get().getRadians(), kDelta);
        assertEquals(Math.PI / 2, delta[2].distanceMeters, kDelta);
        assertEquals(Math.PI, delta[2].angle.get().getRadians(), kDelta);
        assertEquals(0, delta[3].distanceMeters, kDelta);
        assertTrue(delta[3].angle.isEmpty());

        Twist2d twist = kinematics.toTwist2d(delta);

        assertEquals(-0.5 * Math.PI / 2, twist.dx, kDelta);
        assertEquals(0.5 * Math.PI / 2, twist.dy, kDelta);
        assertEquals(Math.PI / 2, twist.dtheta, kDelta);
    }

    @Test
    void testRoll() {
        // a rotate-and-move case you can do in your head
        // face +x with right rear at origin
        // keep origin corner still, rotate around it
        // in this maneuver the steering doesn't change
        SwerveDriveKinematics100 kinematics = new SwerveDriveKinematics100(
                new Translation2d(0.5, 0.5), // 1,1
                new Translation2d(0.5, -0.5), // 1,0
                new Translation2d(-0.5, 0.5), // 0,1
                new Translation2d(-0.5, -0.5) // origin
        );
        Twist2d twist = kinematics.toTwist2d(
                new SwerveModuleDelta(
                        Math.sqrt(2) * Math.PI / 2,
                        Optional.of(Rotation2d.fromRadians(3 * Math.PI / 4))),
                new SwerveModuleDelta(
                        Math.PI / 2,
                        Optional.of(Rotation2d.fromRadians(Math.PI / 2))),
                new SwerveModuleDelta(
                        Math.PI / 2,
                        Optional.of(Rotation2d.fromRadians(Math.PI))),
                // this wheel doesn't move
                new SwerveModuleDelta(
                        0.0,
                        Optional.empty()));

        assertEquals(-0.5 * Math.PI / 2, twist.dx, kDelta);
        assertEquals(0.5 * Math.PI / 2, twist.dy, kDelta);
        assertEquals(Math.PI / 2, twist.dtheta, kDelta);
    }

    @Test
    void testRollInverse() {
        // inverse kinematics for the above case
        SwerveDriveKinematics100 m_kinematics = new SwerveDriveKinematics100(
                new Translation2d(0.5, 0.5),
                new Translation2d(0.5, -0.5),
                new Translation2d(-0.5, 0.5),
                new Translation2d(-0.5, -0.5));

        // move diagonally while turning 90 degrees; this should leave
        // one of the wheels in place.
        Twist2d t = new Twist2d(
                -0.5 * Math.PI / 2,
                0.5 * Math.PI / 2,
                Math.PI / 2);

        // check that the exp is correct
        Pose2d pStart = new Pose2d(0.5, 0.5, GeometryUtil.kRotationZero);
        Pose2d pEnd = pStart.exp(t);
        assertEquals(-0.5, pEnd.getX(), kDelta);
        assertEquals(0.5, pEnd.getY(), kDelta);
        assertEquals(Math.PI / 2, pEnd.getRotation().getRadians(), kDelta);

        // check that the twist is really really correct
        Twist2d t2 = pStart.log(pEnd);
        assertEquals(t, t2);

        SwerveModuleDelta[] p = m_kinematics.toSwerveModuleDelta(t);
        assertEquals(Math.sqrt(2) * Math.PI / 2, p[0].distanceMeters, kDelta);
        assertEquals(3 * Math.PI / 4, p[0].angle.get().getRadians(), kDelta);
        assertEquals(Math.PI / 2, p[1].distanceMeters, kDelta);
        assertEquals(Math.PI / 2, p[1].angle.get().getRadians(), kDelta);
        assertEquals(Math.PI / 2, p[2].distanceMeters, kDelta);
        assertEquals(Math.PI, p[2].angle.get().getRadians(), kDelta);
        // this is the one that shouldn't move
        assertEquals(0, p[3].distanceMeters, kDelta);
        assertTrue(p[3].angle.isEmpty());
    }

    /**
     * array order:
     * 
     * frontLeft
     * frontRight
     * rearLeft
     * rearRight
     */
    @Test
    void testInverse() {
        SwerveDriveKinematics100 kinematics = new SwerveDriveKinematics100(
                new Translation2d(0.5, 0.5),
                new Translation2d(0.5, -0.5),
                new Translation2d(-0.5, 0.5),
                new Translation2d(-0.5, -0.5));
        assertEquals(1, kinematics.m_inverseKinematics.get(0, 0));
        assertEquals(0, kinematics.m_inverseKinematics.get(0, 1));
        assertEquals(-0.5, kinematics.m_inverseKinematics.get(0, 2));
        assertEquals(0, kinematics.m_inverseKinematics.get(1, 0));
        assertEquals(1, kinematics.m_inverseKinematics.get(1, 1));
        assertEquals(0.5, kinematics.m_inverseKinematics.get(1, 2));
        assertEquals(1, kinematics.m_inverseKinematics.get(2, 0));
        assertEquals(0, kinematics.m_inverseKinematics.get(2, 1));
        assertEquals(0.5, kinematics.m_inverseKinematics.get(2, 2));
        assertEquals(0, kinematics.m_inverseKinematics.get(3, 0));
        assertEquals(1, kinematics.m_inverseKinematics.get(3, 1));
        assertEquals(0.5, kinematics.m_inverseKinematics.get(3, 2));
    }

    /**
     * array order:
     * 
     * frontLeft
     * frontRight
     * rearLeft
     * rearRight
     */
    @Test
    void testForward() {
        SwerveDriveKinematics100 kinematics = new SwerveDriveKinematics100(
                new Translation2d(0.5, 0.5),
                new Translation2d(0.5, -0.5),
                new Translation2d(-0.5, 0.5),
                new Translation2d(-0.5, -0.5));
        assertEquals(0.25, kinematics.m_forwardKinematics.get(0, 0), kDelta);
        assertEquals(0, kinematics.m_forwardKinematics.get(1, 0), kDelta);
        assertEquals(-0.25, kinematics.m_forwardKinematics.get(2, 0), kDelta);
        assertEquals(0, kinematics.m_forwardKinematics.get(0, 1), kDelta);
        assertEquals(0.25, kinematics.m_forwardKinematics.get(1, 1), kDelta);
        assertEquals(0.25, kinematics.m_forwardKinematics.get(2, 1), kDelta);
        assertEquals(0.25, kinematics.m_forwardKinematics.get(0, 2), kDelta);
        assertEquals(0, kinematics.m_forwardKinematics.get(1, 2), kDelta);
        assertEquals(0.25, kinematics.m_forwardKinematics.get(2, 2), kDelta);
        assertEquals(0, kinematics.m_forwardKinematics.get(0, 3), kDelta);
        assertEquals(0.25, kinematics.m_forwardKinematics.get(1, 3), kDelta);
        assertEquals(0.25, kinematics.m_forwardKinematics.get(2, 3), kDelta);
    }

    @Test
    void testTwistStraight() {
        SwerveDriveKinematics100 kinematics = new SwerveDriveKinematics100(
                new Translation2d(0.5, 0.5),
                new Translation2d(0.5, -0.5),
                new Translation2d(-0.5, 0.5),
                new Translation2d(-0.5, -0.5));
        // 0.1m straight ahead, all same.
        Twist2d twist = kinematics.toTwist2d(
                new SwerveModuleDelta(0.1, Optional.of(Rotation2d.fromDegrees(0))),
                new SwerveModuleDelta(0.1, Optional.of(Rotation2d.fromDegrees(0))),
                new SwerveModuleDelta(0.1, Optional.of(Rotation2d.fromDegrees(0))),
                new SwerveModuleDelta(0.1, Optional.of(Rotation2d.fromDegrees(0))));

        assertEquals(0.1, twist.dx, kDelta);
        assertEquals(0, twist.dy, kDelta);
        assertEquals(0, twist.dtheta, kDelta);
    }

    @Test
    void testTwistSpin() {
        SwerveDriveKinematics100 kinematics = new SwerveDriveKinematics100(
                new Translation2d(0.5, 0.5),
                new Translation2d(0.5, -0.5),
                new Translation2d(-0.5, 0.5),
                new Translation2d(-0.5, -0.5));

        Twist2d twist = kinematics.toTwist2d(
                new SwerveModuleDelta(0.1, Optional.of(Rotation2d.fromDegrees(135))),
                new SwerveModuleDelta(0.1, Optional.of(Rotation2d.fromDegrees(45))),
                new SwerveModuleDelta(0.1, Optional.of(Rotation2d.fromDegrees(-135))),
                new SwerveModuleDelta(0.1, Optional.of(Rotation2d.fromDegrees(-45))));

        assertEquals(0, twist.dx, kDelta);
        assertEquals(0, twist.dy, kDelta);
        assertEquals(0.141, twist.dtheta, kDelta);
    }

    @Test
    void testWithTime() {
        SwerveDriveKinematics100 kinematics = new SwerveDriveKinematics100(
                new Translation2d(0.5, 0.5),
                new Translation2d(0.5, -0.5),
                new Translation2d(-0.5, 0.5),
                new Translation2d(-0.5, -0.5));
        // 0.1m straight ahead, all same.
        Twist2d twist = kinematics.toTwist2d(
                new SwerveModuleDelta(0.1, Optional.of(Rotation2d.fromDegrees(0))),
                new SwerveModuleDelta(0.1, Optional.of(Rotation2d.fromDegrees(0))),
                new SwerveModuleDelta(0.1, Optional.of(Rotation2d.fromDegrees(0))),
                new SwerveModuleDelta(0.1, Optional.of(Rotation2d.fromDegrees(0))));

        assertEquals(0.1, twist.dx, kDelta);
        assertEquals(0, twist.dy, kDelta);
        assertEquals(0, twist.dtheta, kDelta);
    }

    ////////////////////////////////////////
    //
    // tests below are from WPILib
    //
    //

    private static final double kEpsilon = 1E-9;

    private final Translation2d m_fl = new Translation2d(12, 12);
    private final Translation2d m_fr = new Translation2d(12, -12);
    private final Translation2d m_bl = new Translation2d(-12, 12);
    private final Translation2d m_br = new Translation2d(-12, -12);

    private final SwerveDriveKinematics100 m_kinematics = new SwerveDriveKinematics100(m_fl, m_fr, m_bl, m_br);

    @Test
    void testStraightLineInverseKinematics() { // test inverse kinematics going in a straight line

        ChassisSpeeds speeds = new ChassisSpeeds(5, 0, 0);
        var moduleStates = m_kinematics.toSwerveModuleStates(speeds);

        assertAll(
                () -> assertEquals(5.0, moduleStates[0].speedMetersPerSecond, kEpsilon),
                () -> assertEquals(5.0, moduleStates[1].speedMetersPerSecond, kEpsilon),
                () -> assertEquals(5.0, moduleStates[2].speedMetersPerSecond, kEpsilon),
                () -> assertEquals(5.0, moduleStates[3].speedMetersPerSecond, kEpsilon),
                () -> assertEquals(0.0, moduleStates[0].angle.get().getRadians(), kEpsilon),
                () -> assertEquals(0.0, moduleStates[1].angle.get().getRadians(), kEpsilon),
                () -> assertEquals(0.0, moduleStates[2].angle.get().getRadians(), kEpsilon),
                () -> assertEquals(0.0, moduleStates[3].angle.get().getRadians(), kEpsilon));
    }

    @Test
    void testStraightLineForwardKinematics() { // test forward kinematics going in a straight line
        SwerveModuleState100 state = new SwerveModuleState100(5.0, Optional.of(Rotation2d.fromDegrees(0.0)));
        var chassisSpeeds = m_kinematics.toChassisSpeeds(state, state, state, state);

        assertAll(
                () -> assertEquals(5.0, chassisSpeeds.vxMetersPerSecond, kEpsilon),
                () -> assertEquals(0.0, chassisSpeeds.vyMetersPerSecond, kEpsilon),
                () -> assertEquals(0.0, chassisSpeeds.omegaRadiansPerSecond, kEpsilon));
    }

    @Test
    void testStraightLineForwardKinematicsWithDeltas() {
        // test forward kinematics going in a straight line
        SwerveModuleDelta delta = new SwerveModuleDelta(5.0, Optional.of(Rotation2d.fromDegrees(0.0)));
        Twist2d twist = m_kinematics.toTwist2d(delta, delta, delta, delta);

        assertEquals(5.0, twist.dx, kEpsilon);
        assertEquals(0.0, twist.dy, kEpsilon);
        assertEquals(0.0, twist.dtheta, kEpsilon);
    }

    @Test
    void testStraightStrafeInverseKinematics() {
        ChassisSpeeds speeds = new ChassisSpeeds(0, 5, 0);
        var moduleStates = m_kinematics.toSwerveModuleStates(speeds);

        assertAll(
                () -> assertEquals(5.0, moduleStates[0].speedMetersPerSecond, kEpsilon),
                () -> assertEquals(5.0, moduleStates[1].speedMetersPerSecond, kEpsilon),
                () -> assertEquals(5.0, moduleStates[2].speedMetersPerSecond, kEpsilon),
                () -> assertEquals(5.0, moduleStates[3].speedMetersPerSecond, kEpsilon),
                () -> assertEquals(90.0, moduleStates[0].angle.get().getDegrees(), kEpsilon),
                () -> assertEquals(90.0, moduleStates[1].angle.get().getDegrees(), kEpsilon),
                () -> assertEquals(90.0, moduleStates[2].angle.get().getDegrees(), kEpsilon),
                () -> assertEquals(90.0, moduleStates[3].angle.get().getDegrees(), kEpsilon));
    }

    @Test
    void testStraightStrafeForwardKinematics() {
        SwerveModuleState100 state = new SwerveModuleState100(5.0, Optional.of(Rotation2d.fromDegrees(90.0)));
        var chassisSpeeds = m_kinematics.toChassisSpeeds(state, state, state, state);

        assertAll(
                () -> assertEquals(0.0, chassisSpeeds.vxMetersPerSecond, kEpsilon),
                () -> assertEquals(5.0, chassisSpeeds.vyMetersPerSecond, kEpsilon),
                () -> assertEquals(0.0, chassisSpeeds.omegaRadiansPerSecond, kEpsilon));
    }

    @Test
    void testStraightStrafeForwardKinematicsWithDeltas() {
        SwerveModuleDelta delta = new SwerveModuleDelta(
                5.0,
                Optional.of(Rotation2d.fromDegrees(90.0)));
        Twist2d twist = m_kinematics.toTwist2d(delta, delta, delta, delta);

        assertEquals(0.0, twist.dx, kEpsilon);
        assertEquals(5.0, twist.dy, kEpsilon);
        assertEquals(0.0, twist.dtheta, kEpsilon);
    }

    @Test
    void testConserveWheelAngle() {
        ChassisSpeeds speeds = new ChassisSpeeds(0, 0, 2 * Math.PI);
        m_kinematics.toSwerveModuleStates(speeds);
        var moduleStates = m_kinematics.toSwerveModuleStates(new ChassisSpeeds());

        // Robot is stationary, but module angles are preserved.

        assertAll(
                () -> assertEquals(0.0, moduleStates[0].speedMetersPerSecond, kEpsilon),
                () -> assertEquals(0.0, moduleStates[1].speedMetersPerSecond, kEpsilon),
                () -> assertEquals(0.0, moduleStates[2].speedMetersPerSecond, kEpsilon),
                () -> assertEquals(0.0, moduleStates[3].speedMetersPerSecond, kEpsilon),
                () -> assertEquals(135.0, moduleStates[0].angle.get().getDegrees(), kEpsilon),
                () -> assertEquals(45.0, moduleStates[1].angle.get().getDegrees(), kEpsilon),
                () -> assertEquals(-135.0, moduleStates[2].angle.get().getDegrees(), kEpsilon),
                () -> assertEquals(-45.0, moduleStates[3].angle.get().getDegrees(), kEpsilon));
    }

    @Test
    void testResetWheelAngle() {
        Rotation2d fl = new Rotation2d(0);
        Rotation2d fr = new Rotation2d(Math.PI / 2);
        Rotation2d bl = new Rotation2d(Math.PI);
        Rotation2d br = new Rotation2d(3 * Math.PI / 2);
        m_kinematics.resetHeadings(fl, fr, bl, br);
        var moduleStates = m_kinematics.toSwerveModuleStates(new ChassisSpeeds());

        // Robot is stationary, but module angles are preserved.

        assertAll(
                () -> assertEquals(0.0, moduleStates[0].speedMetersPerSecond, kEpsilon),
                () -> assertEquals(0.0, moduleStates[1].speedMetersPerSecond, kEpsilon),
                () -> assertEquals(0.0, moduleStates[2].speedMetersPerSecond, kEpsilon),
                () -> assertEquals(0.0, moduleStates[3].speedMetersPerSecond, kEpsilon),
                () -> assertEquals(0.0, moduleStates[0].angle.get().getDegrees(), kEpsilon),
                () -> assertEquals(90.0, moduleStates[1].angle.get().getDegrees(), kEpsilon),
                () -> assertEquals(180.0, moduleStates[2].angle.get().getDegrees(), kEpsilon),
                () -> assertEquals(270.0, moduleStates[3].angle.get().getDegrees(), kEpsilon));
    }

    @Test
    void testTurnInPlaceInverseKinematics() {
        ChassisSpeeds speeds = new ChassisSpeeds(0, 0, 2 * Math.PI);
        var moduleStates = m_kinematics.toSwerveModuleStates(speeds);

        /*
         * The circumference of the wheels about the COR is π * diameter, or 2π * radius
         * the radius is the √(12²in + 12²in), or 16.9706in, so the circumference the
         * wheels
         * trace out is 106.629190516in. since we want our robot to rotate at 1 rotation
         * per second,
         * our wheels must trace out 1 rotation (or 106.63 inches) per second.
         */

        assertAll(
                () -> assertEquals(106.63, moduleStates[0].speedMetersPerSecond, 0.1),
                () -> assertEquals(106.63, moduleStates[1].speedMetersPerSecond, 0.1),
                () -> assertEquals(106.63, moduleStates[2].speedMetersPerSecond, 0.1),
                () -> assertEquals(106.63, moduleStates[3].speedMetersPerSecond, 0.1),
                () -> assertEquals(135.0, moduleStates[0].angle.get().getDegrees(), kEpsilon),
                () -> assertEquals(45.0, moduleStates[1].angle.get().getDegrees(), kEpsilon),
                () -> assertEquals(-135.0, moduleStates[2].angle.get().getDegrees(), kEpsilon),
                () -> assertEquals(-45.0, moduleStates[3].angle.get().getDegrees(), kEpsilon));
    }

    @Test
    void testTurnInPlaceForwardKinematics() {
        SwerveModuleState100 flState = new SwerveModuleState100(106.629, Optional.of(Rotation2d.fromDegrees(135)));
        SwerveModuleState100 frState = new SwerveModuleState100(106.629, Optional.of(Rotation2d.fromDegrees(45)));
        SwerveModuleState100 blState = new SwerveModuleState100(106.629, Optional.of(Rotation2d.fromDegrees(-135)));
        SwerveModuleState100 brState = new SwerveModuleState100(106.629, Optional.of(Rotation2d.fromDegrees(-45)));
        var chassisSpeeds = m_kinematics.toChassisSpeeds(flState, frState, blState, brState);

        assertAll(
                () -> assertEquals(0.0, chassisSpeeds.vxMetersPerSecond, kEpsilon),
                () -> assertEquals(0.0, chassisSpeeds.vyMetersPerSecond, kEpsilon),
                () -> assertEquals(2 * Math.PI, chassisSpeeds.omegaRadiansPerSecond, 0.1));
    }

    @Test
    void testTurnInPlaceForwardKinematicsWithDeltas() {
        SwerveModuleDelta flDelta = new SwerveModuleDelta(106.629,
                Optional.of(Rotation2d.fromDegrees(135)));
        SwerveModuleDelta frDelta = new SwerveModuleDelta(106.629,
                Optional.of(Rotation2d.fromDegrees(45)));
        SwerveModuleDelta blDelta = new SwerveModuleDelta(106.629,
                Optional.of(Rotation2d.fromDegrees(-135)));
        SwerveModuleDelta brDelta = new SwerveModuleDelta(106.629,
                Optional.of(Rotation2d.fromDegrees(-45)));

        Twist2d twist = m_kinematics.toTwist2d(flDelta, frDelta, blDelta, brDelta);

        assertEquals(0.0, twist.dx, kEpsilon);
        assertEquals(0.0, twist.dy, kEpsilon);
        assertEquals(2 * Math.PI, twist.dtheta, 0.1);
    }

    @Test
    void testOffCenterCORRotationForwardKinematics() {
        SwerveModuleState100 flState = new SwerveModuleState100(0.0, Optional.of(Rotation2d.fromDegrees(0.0)));
        SwerveModuleState100 frState = new SwerveModuleState100(150.796, Optional.of(Rotation2d.fromDegrees(0.0)));
        SwerveModuleState100 blState = new SwerveModuleState100(150.796, Optional.of(Rotation2d.fromDegrees(-90)));
        SwerveModuleState100 brState = new SwerveModuleState100(213.258, Optional.of(Rotation2d.fromDegrees(-45)));
        var chassisSpeeds = m_kinematics.toChassisSpeeds(flState, frState, blState, brState);

        /*
         * We already know that our omega should be 2π from the previous test. Next, we
         * need to determine
         * the vx and vy of our chassis center. Because our COR is at a 45 degree angle
         * from the center,
         * we know that vx and vy must be the same. Furthermore, we know that the center
         * of mass makes
         * a full revolution about the center of revolution once every second.
         * Therefore, the center of
         * mass must be moving at 106.629in/sec. Recalling that the ratios of a 45/45/90
         * triangle are
         * 1:√(2)/2:√(2)/2, we find that the COM vx is -75.398, and vy is 75.398.
         */

        assertAll(
                () -> assertEquals(75.398, chassisSpeeds.vxMetersPerSecond, 0.1),
                () -> assertEquals(-75.398, chassisSpeeds.vyMetersPerSecond, 0.1),
                () -> assertEquals(2 * Math.PI, chassisSpeeds.omegaRadiansPerSecond, 0.1));
    }

    @Test
    void testOffCenterCORRotationForwardKinematicsWithDeltas() {
        SwerveModuleDelta flDelta = new SwerveModuleDelta(0.0,
                Optional.of(Rotation2d.fromDegrees(0.0)));
        SwerveModuleDelta frDelta = new SwerveModuleDelta(150.796,
                Optional.of(Rotation2d.fromDegrees(0.0)));
        SwerveModuleDelta blDelta = new SwerveModuleDelta(150.796,
                Optional.of(Rotation2d.fromDegrees(-90)));
        SwerveModuleDelta brDelta = new SwerveModuleDelta(213.258,
                Optional.of(Rotation2d.fromDegrees(-45)));

        Twist2d twist = m_kinematics.toTwist2d(flDelta, frDelta, blDelta, brDelta);

        /*
         * We already know that our omega should be 2π from the previous test. Next, we
         * need to determine
         * the vx and vy of our chassis center. Because our COR is at a 45 degree angle
         * from the center,
         * we know that vx and vy must be the same. Furthermore, we know that the center
         * of mass makes
         * a full revolution about the center of revolution once every second.
         * Therefore, the center of
         * mass must be moving at 106.629in/sec. Recalling that the ratios of a 45/45/90
         * triangle are
         * 1:√(2)/2:√(2)/2, we find that the COM vx is -75.398, and vy is 75.398.
         */

        assertAll(
                () -> assertEquals(75.398, twist.dx, 0.1),
                () -> assertEquals(-75.398, twist.dy, 0.1),
                () -> assertEquals(2 * Math.PI, twist.dtheta, 0.1));
    }

    @Test
    void testOffCenterCORRotationAndTranslationForwardKinematics() {
        SwerveModuleState100 flState = new SwerveModuleState100(23.43, Optional.of(Rotation2d.fromDegrees(-140.19)));
        SwerveModuleState100 frState = new SwerveModuleState100(23.43, Optional.of(Rotation2d.fromDegrees(-39.81)));
        SwerveModuleState100 blState = new SwerveModuleState100(54.08, Optional.of(Rotation2d.fromDegrees(-109.44)));
        SwerveModuleState100 brState = new SwerveModuleState100(54.08, Optional.of(Rotation2d.fromDegrees(-70.56)));
        var chassisSpeeds = m_kinematics.toChassisSpeeds(flState, frState, blState, brState);

        /*
         * From equation (13.17), we know that chassis motion is th dot product of the
         * pseudoinverse of the inverseKinematics matrix (with the center of rotation at
         * (0,0) -- we don't want the motion of the center of rotation, we want it of
         * the center of the robot). These above SwerveModuleState100s are known to be
         * from
         * a velocity of [[0][3][1.5]] about (0, 24), and the expected numbers have been
         * calculated using Numpy's linalg.pinv function.
         */

        assertEquals(0.0, chassisSpeeds.vxMetersPerSecond, 0.1);
        assertEquals(-33.0, chassisSpeeds.vyMetersPerSecond, 0.1);
        assertEquals(1.5, chassisSpeeds.omegaRadiansPerSecond, 0.1);
    }

    @Test
    void testOffCenterCORRotationAndTranslationForwardKinematicsWithDeltas() {
        SwerveModuleDelta flDelta = new SwerveModuleDelta(23.43,
                Optional.of(Rotation2d.fromDegrees(-140.19)));
        SwerveModuleDelta frDelta = new SwerveModuleDelta(23.43,
                Optional.of(Rotation2d.fromDegrees(-39.81)));
        SwerveModuleDelta blDelta = new SwerveModuleDelta(54.08,
                Optional.of(Rotation2d.fromDegrees(-109.44)));
        SwerveModuleDelta brDelta = new SwerveModuleDelta(54.08,
                Optional.of(Rotation2d.fromDegrees(-70.56)));

        Twist2d twist = m_kinematics.toTwist2d(flDelta, frDelta, blDelta, brDelta);

        /*
         * From equation (13.17), we know that chassis motion is th dot product of the
         * pseudoinverse of the inverseKinematics matrix (with the center of rotation at
         * (0,0) -- we don't want the motion of the center of rotation, we want it of
         * the center of the robot). These above SwerveModuleState100s are known to be
         * from
         * a velocity of [[0][3][1.5]] about (0, 24), and the expected numbers have been
         * calculated using Numpy's linalg.pinv function.
         */

        assertAll(
                () -> assertEquals(0.0, twist.dx, 0.1),
                () -> assertEquals(-33.0, twist.dy, 0.1),
                () -> assertEquals(1.5, twist.dtheta, 0.1));
    }

    @Test
    void testModuleKinematics() {
        SwerveDriveKinematics100 kinematics = new SwerveDriveKinematics100(
                new Translation2d(0.5, 0.5),
                new Translation2d(0.5, -0.5),
                new Translation2d(-0.5, 0.5),
                new Translation2d(-0.5, -0.5));
        SimpleMatrix accelerations = new SimpleMatrix(3, 1);
        SimpleMatrix expected = new SimpleMatrix(2, 1);
        accelerations.setColumn(0, 0, 0, 0, 0);
        expected.setColumn(0, 0, 0, 0);
        SimpleMatrix output = kinematics.getModuleAccelerationXY(0, accelerations);
        Util.println(output.get(0, 0) + " " + output.get(1, 0));
        assertEquals(0, output.get(0, 0), 0.0001);
        assertEquals(0, output.get(1, 0), 0.0001);
    }

    @Test
    void testModuleKinematics2() {
        // one-meter drive base
        SwerveDriveKinematics100 kinematics = new SwerveDriveKinematics100(
                new Translation2d(0.5, 0.5),
                new Translation2d(0.5, -0.5),
                new Translation2d(-0.5, 0.5),
                new Translation2d(-0.5, -0.5));

        SwerveModuleState100[] prev = {
                new SwerveModuleState100(0, Optional.of(new Rotation2d())),
                new SwerveModuleState100(0, Optional.of(new Rotation2d())),
                new SwerveModuleState100(0, Optional.of(new Rotation2d())),
                new SwerveModuleState100(0, Optional.of(new Rotation2d())) };

        {
            SimpleMatrix v = new SimpleMatrix(3, 1);
            v.setColumn(0, 0, 0, 0, 1);
            SwerveModuleState100[] output2 = kinematics.statesFromVector(v);

            SimpleMatrix a = new SimpleMatrix(3, 1);
            a.setColumn(0, 0, 0, 0, 0);
            SwerveModuleState100[] output = kinematics.accelerationFromVector(v, a, prev);

            Util.println(output[0].toString());

            assertEquals(0, output[0].accelMetersPerSecond_2, 0.0001);
            for (int i = 0; i < output.length; i++) {
                assertEquals(output[i].angle, output2[i].angle);
                assertEquals(output[i].speedMetersPerSecond, output2[i].speedMetersPerSecond);
            }
        }

        {
            SimpleMatrix v = new SimpleMatrix(3, 1);
            v.setColumn(0, 0, 2, 1, 1);
            SwerveModuleState100[] output4 = kinematics.statesFromVector(v);
            SimpleMatrix a = new SimpleMatrix(3, 1);
            a.setColumn(0, 0, 2, 2, 2);
            SwerveModuleState100[] output3 = kinematics.accelerationFromVector(v, a, prev);
            for (int i = 0; i < output3.length; i++) {
                assertEquals(output3[i].angle, output4[i].angle);
                assertEquals(output3[i].speedMetersPerSecond, output4[i].speedMetersPerSecond);
            }
        }

        {
            SimpleMatrix v = new SimpleMatrix(3, 1);
            v.setColumn(0, 0, 2.3, 0, 1.1);

            SwerveModuleState100[] output5 = kinematics.statesFromVector(v);

            SimpleMatrix a = new SimpleMatrix(3, 1);
            a.setColumn(0, 0, 1.32, 1.2, 2.2);

            SwerveModuleState100[] output6 = kinematics.accelerationFromVector(v, a, prev);

            for (int i = 0; i < output5.length; i++) {
                assertEquals(output5[i].angle, output6[i].angle);
                assertEquals(output5[i].speedMetersPerSecond, output6[i].speedMetersPerSecond);
            }
        }

        {
            // motionless
            SimpleMatrix v = new SimpleMatrix(3, 1);
            v.setColumn(0, 0, 0, 0, 0);
            SwerveModuleState100[] output7 = kinematics.statesFromVector(v);
            SimpleMatrix a = new SimpleMatrix(3, 1);
            a.setColumn(0, 0, 1, 1.1, 2);
            SwerveModuleState100[] output8 = kinematics.accelerationFromVector(v, a, prev);
            for (int i = 0; i < output7.length; i++) {
                assertEquals(output7[i].angle, output8[i].angle);
                assertEquals(output7[i].speedMetersPerSecond, output8[i].speedMetersPerSecond);
            }
        }
    }

    @Test
    void testModuleKinematics3a() {
        SwerveDriveKinematics100 kinematics = new SwerveDriveKinematics100(
                new Translation2d(0.5, 0.5),
                new Translation2d(0.5, -0.5),
                new Translation2d(-0.5, 0.5),
                new Translation2d(-0.5, -0.5));
        SwerveModuleState100[] prevStates = {
                new SwerveModuleState100(0, Optional.of(new Rotation2d())),
                new SwerveModuleState100(0, Optional.of(new Rotation2d())),
                new SwerveModuleState100(0, Optional.of(new Rotation2d())),
                new SwerveModuleState100(0, Optional.of(new Rotation2d())) };

        // no velocity
        SimpleMatrix velocities = new SimpleMatrix(3, 1);
        velocities.setColumn(0, 0, 0, 0, 0);

        // accelerating in x i guess
        SimpleMatrix acceleration = new SimpleMatrix(3, 1);
        acceleration.setColumn(0, 0, 1, 0, 0);

        SwerveModuleState100[] output = kinematics.accelerationFromVector(
                velocities,
                acceleration,
                prevStates);

        for (SwerveModuleState100 state : output) {
            // assertEquals(state.angle.get().getRadians(), 0, 0.0001);
            // since the module isn't yet moving it has an indeterminate angle.
            assertTrue(state.angle.isEmpty());
            assertEquals(0, state.omega, 0.0001);
            assertEquals(0, state.speedMetersPerSecond, 0.0001);
            // TODO: fix this test
            // assertEquals(state.accelMetersPerSecond_2, 1, 0.0001);
        }

    }

    @Test
    void testModuleKinematics3b() {
        SwerveDriveKinematics100 kinematics = new SwerveDriveKinematics100(
                new Translation2d(0.5, 0.5),
                new Translation2d(0.5, -0.5),
                new Translation2d(-0.5, 0.5),
                new Translation2d(-0.5, -0.5));
        SwerveModuleState100[] prevStates = {
                new SwerveModuleState100(0, Optional.of(new Rotation2d())),
                new SwerveModuleState100(0, Optional.of(new Rotation2d())),
                new SwerveModuleState100(0, Optional.of(new Rotation2d())),
                new SwerveModuleState100(0, Optional.of(new Rotation2d())) };

        SimpleMatrix velocities = new SimpleMatrix(3, 1);
        SimpleMatrix acceleration = new SimpleMatrix(3, 1);
        SwerveModuleState100[] output = new SwerveModuleState100[4];

        velocities.setColumn(0, 0, 1, 0, 0);
        acceleration.setColumn(0, 0, 1, 0, 0);

        output = kinematics.accelerationFromVector(velocities, acceleration, prevStates);

        for (SwerveModuleState100 state : output) {
            assertEquals(0, state.angle.get().getRadians(), 0.0001);
            assertEquals(0, state.omega, 0.0001);
            assertEquals(1, state.speedMetersPerSecond, 0.0001);
            assertEquals(1, state.accelMetersPerSecond_2, 0.0001);
        }

    }

    @Test
    void testModuleKinematics3c() {
        SwerveDriveKinematics100 kinematics = new SwerveDriveKinematics100(
                new Translation2d(0.5, 0.5),
                new Translation2d(0.5, -0.5),
                new Translation2d(-0.5, 0.5),
                new Translation2d(-0.5, -0.5));
        SwerveModuleState100[] prevStates = {
                new SwerveModuleState100(0, Optional.of(new Rotation2d())),
                new SwerveModuleState100(0, Optional.of(new Rotation2d())),
                new SwerveModuleState100(0, Optional.of(new Rotation2d())),
                new SwerveModuleState100(0, Optional.of(new Rotation2d())) };

        SimpleMatrix velocities = new SimpleMatrix(3, 1);
        SimpleMatrix acceleration = new SimpleMatrix(3, 1);
        SwerveModuleState100[] output = new SwerveModuleState100[4];

        velocities.setColumn(0, 0, 1, 0, 0);
        acceleration.setColumn(0, 0, 0, 0, 1);
        output = kinematics.accelerationFromVector(velocities,
                acceleration, prevStates);

        int count = 0;
        for (SwerveModuleState100 state : output) {
            assertEquals(0, state.angle.get().getRadians(), 0.0001);
            assertEquals(1, state.speedMetersPerSecond, 0.0001);
            switch (count) {
                case 0:
                    assertEquals(0.5, state.omega, 0.0001);
                    assertEquals(-0.5, state.accelMetersPerSecond_2, 0.0001);
                    break;
                case 1:
                    assertEquals(0.5, state.omega, 0.0001);
                    assertEquals(0.5, state.accelMetersPerSecond_2, 0.0001);
                    break;
                case 2:
                    assertEquals(-0.5, state.omega, 0.0001);
                    assertEquals(-0.5, state.accelMetersPerSecond_2, 0.0001);
                    break;
                case 3:
                    assertEquals(state.omega, -0.5, 0.0001);
                    assertEquals(0.5, state.accelMetersPerSecond_2, 0.0001);
                    break;
                default:
                    throw new UnsupportedOperationException("Not a swerve module");
            }
            count++;
        }

    }

    @Test
    void testDesaturate() {
        SwerveModuleState100 fl = new SwerveModuleState100(5, Optional.of(new Rotation2d()));
        SwerveModuleState100 fr = new SwerveModuleState100(6, Optional.of(new Rotation2d()));
        SwerveModuleState100 bl = new SwerveModuleState100(4, Optional.of(new Rotation2d()));
        SwerveModuleState100 br = new SwerveModuleState100(7, Optional.of(new Rotation2d()));

        SwerveModuleState100[] arr = { fl, fr, bl, br };
        SwerveDriveKinematics100.desaturateWheelSpeeds(arr, 5.5);

        double factor = 5.5 / 7.0;

        assertAll(
                () -> assertEquals(5.0 * factor, arr[0].speedMetersPerSecond, kEpsilon),
                () -> assertEquals(6.0 * factor, arr[1].speedMetersPerSecond, kEpsilon),
                () -> assertEquals(4.0 * factor, arr[2].speedMetersPerSecond, kEpsilon),
                () -> assertEquals(7.0 * factor, arr[3].speedMetersPerSecond, kEpsilon));
    }

    @Test
    void testDesaturateNegativeSpeed() {
        SwerveModuleState100 fl = new SwerveModuleState100(1, Optional.of(new Rotation2d()));
        SwerveModuleState100 fr = new SwerveModuleState100(1, Optional.of(new Rotation2d()));
        SwerveModuleState100 bl = new SwerveModuleState100(-2, Optional.of(new Rotation2d()));
        SwerveModuleState100 br = new SwerveModuleState100(-2, Optional.of(new Rotation2d()));

        SwerveModuleState100[] arr = { fl, fr, bl, br };
        SwerveDriveKinematics100.desaturateWheelSpeeds(arr, 1);

        assertAll(
                () -> assertEquals(0.5, arr[0].speedMetersPerSecond, kEpsilon),
                () -> assertEquals(0.5, arr[1].speedMetersPerSecond, kEpsilon),
                () -> assertEquals(-1.0, arr[2].speedMetersPerSecond, kEpsilon),
                () -> assertEquals(-1.0, arr[3].speedMetersPerSecond, kEpsilon));
    }
}
