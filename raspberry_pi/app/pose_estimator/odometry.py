"""Factory for odometry factors.

This also serves as a template for other factors.  The pattern should always be
the same: an "h" function that performs the estimated measurement, an
"h_H" function that wraps it with numerical derivatives (to avoid confusion
with analytic Jacobians), and a "factor" factory that wraps that in a
CustomFactor.

There's some discussion relevant to these operations here:
https://groups.google.com/g/gtsam-users/c/c-BhH8mfqbo/m/IMk1RQ84AwAJ
"""

# pylint: disable=C0103,E0611

import gtsam
import numpy as np
from gtsam.noiseModel import Base as SharedNoiseModel
from wpimath.geometry import Twist2d

from app.pose_estimator.numerical_derivative import (
    numericalDerivative21,
    numericalDerivative22,
)


def h(p0: gtsam.Pose2, p1: gtsam.Pose2) -> np.ndarray:
    """Difference between p0 and p1 in the tangent space.
    This is identical to the WPILib "Twist2d" idea. """
    return p0.logmap(p1)


def h_H(measured: np.ndarray, p0: gtsam.Pose2, p1: gtsam.Pose2, H: list[np.ndarray]):
    """Error function including Jacobians.
    Returns the difference between the measured and estimated tangent-space odometry."""
    result = h(p0, p1) - measured
    if H is not None:
        H[0] = numericalDerivative21(h, p0, p1)
        H[1] = numericalDerivative22(h, p0, p1)
    return result


def factor(
    t: Twist2d,
    model: SharedNoiseModel,
    p0_key: gtsam.Symbol,
    p1_key: gtsam.Symbol,
) -> gtsam.NonlinearFactor:
    """Factory for Custom Factor implementing odometry using tangent-space measurements.
    """
    measured = np.array([t.dx, t.dy, t.dtheta])

    def error_func(
        this: gtsam.CustomFactor, v: gtsam.Values, H: list[np.ndarray]
    ) -> np.ndarray:
        p0: gtsam.Pose2 = v.atPose2(this.keys()[0])
        p1: gtsam.Pose2 = v.atPose2(this.keys()[1])
        return h_H(measured, p0, p1, H)

    return gtsam.CustomFactor(model, gtsam.KeyVector([p0_key, p1_key]), error_func)
