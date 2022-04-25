import pytest
from safety_shield_py import Motion


class TestMotionConstruction:
    def test_constructor1(self):
        m = Motion()
        assert m is not None
        assert m.getTime() == 0.0

    def test_constructor2(self):
        m = Motion(1)
        assert m is not None
        assert m.getTime() == 0.0

    def test_constructor3(self):
        m = Motion(1.0, [0.0, 0.0, 1.0], 1.0)
        assert m is not None
        assert m.getTime() == 1.0

    def test_constructor3_2(self):
        m = Motion(1.0, [0.0, 0.0, 1.0])
        assert m is not None
        assert m.getTime() == 1.0

    def test_constructor4(self):
        m = Motion(4.0, [0.0, 0.0, 1.0], [0.0, 0.0, 0.0])
        assert m is not None
        assert m.getTime() == 4.0

    def test_constructor5(self):
        m = Motion(5.0, [0.0, 0.0, 1.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
        assert m is not None
        assert m.getTime() == 5.0

    def test_constructor6(self):
        m = Motion(
            6.0, [0.0, 0.0, 1.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]
        )
        assert m is not None
        assert m.getTime() == 6.0


class TestMotionFunctions:
    @pytest.fixture
    def full_motion(self):
        return Motion(
            1.0, [1.0, 2.0, 3.0], [1.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 1.0, 0.0], 2.0
        )

    def test_getTime(self, full_motion):
        assert full_motion.getTime() == 1.0

    def test_getS(self, full_motion):
        assert full_motion.getS() == 2.0

    def test_getAngle(self, full_motion):
        assert full_motion.getAngle() == [1.0, 2.0, 3.0]

    def test_getVelocity(self, full_motion):
        assert full_motion.getVelocity() == [1.0, 0.0, 0.0]

    def test_getAcceleration(self, full_motion):
        assert full_motion.getAcceleration() == [0.0, 0.0, 0.0]

    def test_getJerk(self, full_motion):
        assert full_motion.getJerk() == [0.0, 1.0, 0.0]

    def test_setTime(self, full_motion):
        full_motion.setTime(2.0)
        assert full_motion.getTime() == 2.0

    def test_setS(self, full_motion):
        full_motion.setS(3.0)
        assert full_motion.getS() == 3.0

    def test_setAngle(self, full_motion):
        full_motion.setAngle([0.0, 0.0, 0.0])
        assert full_motion.getAngle() == [0.0, 0.0, 0.0]

    def test_setVelocity(self, full_motion):
        full_motion.setVelocity([-1.0, -1.0, 1.0])
        assert full_motion.getVelocity() == [-1.0, -1.0, 1.0]

    def test_setAcceleration(self, full_motion):
        full_motion.setAcceleration([0.0, 0.0, 1.0])
        assert full_motion.getAcceleration() == [0.0, 0.0, 1.0]

    def test_setJerk(self, full_motion):
        full_motion.setJerk([0.0, 0.0, 0.0])
        assert full_motion.getJerk() == [0.0, 0.0, 0.0]
