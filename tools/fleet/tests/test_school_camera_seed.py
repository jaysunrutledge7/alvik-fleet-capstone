"""9 Nov 2026, school: the first real school free roam crashed seeding the readability map, because the real camera
model's project() takes one point and returns (u_array, v_array) for a list. mission_run now projects one point at a
time; this checks the seed works with a real CameraModel (a synthetic overhead camera)."""
import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [os.path.join(HERE, ".."), os.path.join(HERE, "..", "..", "recognition")]
cv2 = pytest.importorskip("cv2")
from cameras import CameraModel  # noqa: E402
import observe as OB  # noqa: E402


class _RM:
    def seed(self, fn, why):
        self.v = [fn(x, y) for x in range(0, 130, 10) for y in range(0, 130, 10)]

    def grow(self, n):
        pass


def _cam():
    K = np.array([[600, 0, 320], [0, 600, 240], [0, 0, 1.0]])
    R = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1.0]])
    rvec, _ = cv2.Rodrigues(R)
    return CameraModel(K, np.zeros(5), rvec.ravel(), -R @ np.array([60, 60, 145.0]), (640, 480))


def test_seed_with_a_real_camera_model_one_point_at_a_time():
    m = _cam()
    proj = lambda P, m=m: [tuple(m.project(tuple(q))) for q in P]   # noqa: E731  (as mission_run builds it)
    rm = _RM()
    OB.seed_from_cameras(rm, [(proj, (640, 480))], 5, 6)
    assert 0 < sum(v > 0.5 for v in rm.v) < len(rm.v)


def test_the_list_call_was_the_crash():
    m = _cam()
    with pytest.raises(IndexError):
        OB.seed_from_cameras(_RM(), [(lambda P, m=m: m.project(P), (640, 480))], 5, 6)
