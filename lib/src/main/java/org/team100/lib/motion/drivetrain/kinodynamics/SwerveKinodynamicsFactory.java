package org.team100.lib.motion.drivetrain.kinodynamics;

import org.team100.lib.config.Identity;
import org.team100.lib.telemetry.SupplierLogger;
import org.team100.lib.util.Tire;
import org.team100.lib.util.Util;

/**
 * Each drivetrain should be tuned, and the values here should be the physical
 * maxima.
 * 
 * FYI according to their 2022 code, 254's max speed in 2022 was 5.05 m/s, which
 * is about the same as ours, but their max acceleration was 4.4 m/s^2, which is
 * crazy quick.
 *
 * Tune these limits to match the absolute maximum possible performance of the
 * drivetrain, not what seems "comfortable."
 * 
 * Do not use this class to configure driver preferences, use a command or
 * control instead.
 * 
 * In particular, the maximum spin rate is likely to seem quite high. Do not
 * lower it here.
 */
public class SwerveKinodynamicsFactory {
    public static SwerveKinodynamics get(SupplierLogger parent) {
        switch (Identity.instance) {
            case COMP_BOT:
                // these numbers are a guess based on the betabot numbers.
                // the comp bot uses the "fast" ratio and FOC falcons
                // so should be a bit higher top speed and less acceleration.
                // note these measurements were updated jun 24.
                return new SwerveKinodynamics(
                        parent,
                        5, // max vel m/s
                        10, // stall m/s/s
                        10, // max accel m/s/s
                        20, // max decel m/s/s
                        20, // max module steering rate rad/s
                        60, // max module steering accel rad/s/s
                        0.48, // front track m
                        0.43, // back track m
                        0.46, // wheelbase m
                        0.31, // front offset m
                        0.1, // vcg m
                        Tire.noslip(parent));
            case SWERVE_TWO:
                return new SwerveKinodynamics(
                        parent,
                        4, // vel m/s
                        10, // stall m/s/s
                        2, // accel m/s/s
                        2, // decel m/s/s
                        13, // steering rate rad/s
                        20 * Math.PI, // steering accel rad/s/s
                        0.380, // track m 
                        0.445, // wheelbase m
                        0.2225, // front offset m
                        0.3, // vcg m
                        Tire.noslip(parent));
            case SWERVE_ONE:
                return new SwerveKinodynamics(
                        parent,
                        4, // vel m/s
                        10, // stall m/s/s
                        2, // accel m/s/s
                        2, // decel m/s/s
                        13, // steering rate rad/s
                        20 * Math.PI, // steering accel rad/s/s
                        0.449, // track m
                        0.464, // wheelbase m
                        .232, // front offset m
                        0.3, // vcg m
                        Tire.noslip(parent));
            case BLANK:
                // this is used for tests and simulation; if you change it you should fix all
                // the broken tests.
                return new SwerveKinodynamics(
                        parent,
                        4, // vel m/s
                        10, // stall m/s/s
                        4, // accel m/s/s
                        4, // decel m/s/s
                        13, // steering rate rad/s
                        20 * Math.PI, // steering accel rad/s/s
                        0.5, // track m
                        0.5, // wheelbase m
                        .25, // front offset m
                        0.3, // vcg m
                        Tire.noslip(parent));
            case BETA_BOT:
                // these numbers were extracted from module mode acceleration
                // runs as shown in this spreadsheet
                // https://docs.google.com/spreadsheets/d/1x0WEDIYosVBrsz37VXPEEmLB6-AuLnmwBp_mgozKFI0
                // the actual profile is exponential. these numbers represent the maximum
                // tangent
                // so that the result will be snappy at low speed, and unable to meet its
                // setpoints at high speed.
                // note the betabot uses the "medium" speed ratio
                // and falcons with FOC -- these data were taken when the gear ratio was
                // misconfigured so i reduced them accordingly.
                // also i observed the steering speed and reduced it a bit.
                // the beta bot has very low VCG.
                return new SwerveKinodynamics(
                        parent,
                        5, // max vel m/s
                        20, // max accel m/s/s
                        50, // max decel m/s/s
                        20, // max module steering rate rad/s
                        60, // max module steering accel rad/s/s
                        0.491, // front track m
                        0.44, // back track m
                        0.491, // wheelbase m
                        0.29, // front offset m
                        0.1, // vcg m
                        Tire.noslip(parent));
            default:
                Util.warn("Using default kinodynamics");
                return new SwerveKinodynamics(
                        parent,
                        5, // vel m/s
                        20, // stall m/s/s
                        5, // accel m/s/s
                        5, // decel m/s/s
                        13, // steering rate rad/s
                        20 * Math.PI, // steering accel rad/s/s
                        0.5, // track m
                        0.5, // wheelbase m
                        .25, // front offset m
                        0.3, // vcg m
                        Tire.noslip(parent));
        }
    }

    /**
     * This contains garbage values, not for anything real.
     * 
     * In particular, the steering rate is *very* slow, which might be useful if
     * you're wanting to allow for steering delay.
     */
    public static SwerveKinodynamics forTest(SupplierLogger parent) {
        return new SwerveKinodynamics(
                parent,
                1, // vel m/s
                10, // stall m/s/s
                1, // accel m/s/s
                1, // decel m/s/s
                20 * Math.PI,
                20 * Math.PI, // steering accel rad/s/s
                0.5, // track m
                0.5, // wheelbase m
                .25, // front offset m
                0.3, // vcg m
                Tire.noslip(parent));
    }

    public static SwerveKinodynamics forTestWithSlip(SupplierLogger parent) {
        return new SwerveKinodynamics(
                parent,
                1, // vel m/s
                10, // stall m/s/s
                1, // accel m/s/s
                1, // decel m/s/s
                20 * Math.PI, // steering rate rad/s
                20 * Math.PI, // steering accel rad/s/s
                0.5, // track m
                0.5, // wheelbase m
                .25, // front offset m
                0.3, // vcg m
                Tire.defaultTire(parent));
    }

    public static SwerveKinodynamics forTest2(SupplierLogger parent) {
        return new SwerveKinodynamics(
                parent,
                2, // vel m/s
                5, // stall m/s/s
                1, // accel m/s/s
                1, // decel m/s/s
                1, // steering rate rad/s
                20 * Math.PI, // steering accel rad/s/s
                0.5, // track m
                0.5, // wheelbase m
                .25, // front offset m
                0.6, // vcg m
                Tire.noslip(parent));
    }

    public static SwerveKinodynamics forWPITest(SupplierLogger parent) {
        return new SwerveKinodynamics(
                parent,
                1, // vel m/s
                5, // stall m/s/s
                1, // accel m/s/s
                1, // decel m/s/s
                1, // steering rate rad/s
                1, // steering accel rad/s/s
                2, // track m
                2, // wheelbase m
                1, // front offset m
                1, // vcg m
                Tire.noslip(parent));
    }
    //////////////////////////////////////////
    //
    // below are specific test cases. try to minimize their number

    public static SwerveKinodynamics highDecelAndCapsize(SupplierLogger parent) {
        return new SwerveKinodynamics(
                parent,
                5, // vel m/s
                10, // stall m/s/s
                2, // accel m/s/s
                300, // decel m/s/s
                5, // steering rate rad/s
                20 * Math.PI, // steering accel rad/s/s
                0.5, // track m
                0.5, // wheelbase m
                .25, // front offset m
                0.001, // vcg m
                Tire.noslip(parent));
    }

    public static SwerveKinodynamics decelCase(SupplierLogger parent) {
        return new SwerveKinodynamics(
                parent,
                1, // vel m/s
                10, // stall m/s/s
                1, // accel m/s/s
                10, // decel m/s/s
                5, // steering rate rad/s
                20 * Math.PI, // steering accel rad/s/s
                0.5, // track m
                0.5, // wheelbase m
                .25, // front offset m
                0.3, // vcg m
                Tire.noslip(parent));
    }

    public static SwerveKinodynamics highCapsize(SupplierLogger parent) {
        return new SwerveKinodynamics(
                parent,
                5, // vel m/s
                20, // stall m/s/s
                10, // accel m/s/s
                10, // decel m/s/s
                5, // steering rate rad/s
                20 * Math.PI, // steering accel rad/s/s
                0.5, // track m
                0.5, // wheelbase m
                .25, // front offset m
                0.1, // vcg m
                Tire.noslip(parent));
    }

    public static SwerveKinodynamics lowCapsize(SupplierLogger parent) {
        return new SwerveKinodynamics(
                parent,
                5, // vel m/s
                20, // stall m/s/s
                10, // accel m/s/s
                10, // decel m/s/s
                5, // steering rate rad/s
                20 * Math.PI, // steering accel rad/s/s
                0.5, // track m
                0.5, // wheelbase m
                .25, // front offset m
                2, // vcg m (very high vcg)
                Tire.noslip(parent));
    }

    public static SwerveKinodynamics limiting(SupplierLogger parent) {
        return new SwerveKinodynamics(
                parent,
                5, // vel m/s
                30, // stall m/s/s
                10, // accel m/s/s
                10, // decel m/s/s
                5, // steering rate rad/s
                20 * Math.PI, // steering accel rad/s/s
                0.5, // track m
                0.5, // wheelbase m
                .25, // front offset m
                0.3, // vcg m
                Tire.noslip(parent));
    }

    /** Large difference in accel and decel, to make asymmetry obvious. */
    public static SwerveKinodynamics lowAccelHighDecel(SupplierLogger parent) {
        return new SwerveKinodynamics(
                parent,
                4, // vel m/s
                10, // stall m/s/s
                1, // accel m/s/s
                10, // decel m/s/s
                5, // steering rate rad/s
                20 * Math.PI, // steering accel rad/s/s
                0.5, // track m
                0.5, // wheelbase m
                .25, // front offset m
                0.3, // vcg m
                Tire.noslip(parent));
    }

    private SwerveKinodynamicsFactory() {
        //
    }
}
