"""Evaluate the estimator for accelerometry."""

# pylint: disable=C0301,E0611,E1101,R0903

import unittest

import gtsam
from gtsam.symbol_shorthand import X
from wpimath.geometry import Pose2d

from app.pose_estimator.estimate import Estimate


class EstimateAccelerometerTest(unittest.TestCase):
    def test_accelerometer_0(self) -> None:
        est = Estimate()
        est.init(Pose2d(0, 0, 0))
        est.add_state(20000, gtsam.Pose2(0.02, 0, 0))
        est.add_state(40000, gtsam.Pose2(0.04, 0, 0))
        est.accelerometer(0, 20000, 40000, 0, 0)
        est.update()
        print(est.result)
        self.assertEqual(3, est.result.size())
        p0: gtsam.Pose2 = est.result.atPose2(X(0))
        self.assertAlmostEqual(0, p0.x())
        self.assertAlmostEqual(0, p0.y())
        self.assertAlmostEqual(0, p0.theta())
        p1: gtsam.Pose2 = est.result.atPose2(X(20000))
        self.assertAlmostEqual(0.02, p1.x())
        self.assertAlmostEqual(0, p1.y())
        self.assertAlmostEqual(0, p1.theta())
        p2: gtsam.Pose2 = est.result.atPose2(X(40000))
        self.assertAlmostEqual(0.04, p2.x())
        self.assertAlmostEqual(0, p2.y())
        self.assertAlmostEqual(0, p2.theta())

    def test_accelerometer_1(self) -> None:
        est = Estimate()
        est.init(Pose2d(0, 0, 0))
        est.add_state(20000, gtsam.Pose2(0.02, 0, 0))
        est.add_state(40000, gtsam.Pose2(0.04, 0, 0))
        # non-zero accel
        est.accelerometer(0, 20000, 40000, 1, 0)
        est.update()
        print(est.result)
        self.assertEqual(3, est.result.size())
        p0: gtsam.Pose2 = est.result.atPose2(X(0))
        # this state has a prior so it is relatively immobile
        self.assertAlmostEqual(0, p0.x())
        self.assertAlmostEqual(0, p0.y())
        self.assertAlmostEqual(0, p0.theta())
        p1: gtsam.Pose2 = est.result.atPose2(X(20000))
        # accel nudges this state back a bit
        self.assertAlmostEqual(0.0198, p1.x(), 3)
        self.assertAlmostEqual(0, p1.y())
        self.assertAlmostEqual(0, p1.theta())
        # accel nudges this state forward a bit
        p2: gtsam.Pose2 = est.result.atPose2(X(40000))
        self.assertAlmostEqual(0.04008, p2.x(),5 )
        self.assertAlmostEqual(0, p2.y())
        self.assertAlmostEqual(0, p2.theta())
