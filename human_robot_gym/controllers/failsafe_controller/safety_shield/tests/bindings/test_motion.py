"""This file defines the tests of the python bindings of the motion class.

Owner:
    Jakob Thumm (JT)

Contributors:

Changelog:
    2.5.22 JT Formatted docstrings
    5.5.22 JT Added more tests
"""

import pytest
from safety_shield_py import Motion


class TestMotionConstruction:
    """This class defines the tests of the py bindings of the motion class."""

    def test_constructor1(self):
        """Test the motion constructor 1."""
        m = Motion()
        assert m is not None
        assert m.getTime() == 0.0

    def test_constructor2(self):
        """Test the motion constructor 2."""
        m = Motion(1)
        assert m is not None
        assert m.getTime() == 0.0

    def test_constructor3(self):
        """Test the motion constructor 3."""
        m = Motion(1.0, [0.0, 0.0, 1.0], 1.0)
        assert m is not None
        assert m.getTime() == 1.0

    def test_constructor3_2(self):
        """Test the motion constructor 3."""
        m = Motion(1.0, [0.0, 0.0, 1.0])
        assert m is not None
        assert m.getTime() == 1.0

    def test_constructor4(self):
        """Test the motion constructor 4."""
        m = Motion(4.0, [0.0, 0.0, 1.0], [0.0, 0.0, 0.0])
        assert m is not None
        assert m.getTime() == 4.0

    def test_constructor5(self):
        """Test the motion constructor 5."""
        m = Motion(5.0, [0.0, 0.0, 1.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
        assert m is not None
        assert m.getTime() == 5.0

    def test_constructor6(self):
        """Test the motion constructor 6."""
        m = Motion(
            6.0, [0.0, 0.0, 1.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]
        )
        assert m is not None
        assert m.getTime() == 6.0


class TestMotionFunctions:
    """This class defines a test fixture for the motion tests and the tests itself."""

    @pytest.fixture
    def full_motion(self):
        """Define a test fixture for the motion class.

        Defines pos, vel, acc, and jerk.
        """
        return Motion(
            1.0, [1.0, 2.0, 3.0], [1.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 1.0, 0.0], 2.0
        )

    @pytest.fixture
    def stopped_motion(self):
        """Define a test fixture for the motion class (stopped).

        Defines pos, vel, acc, and jerk.
        """
        return Motion(
            0.0, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], 0.0
        )

    def test_isStoppedFalse(self, full_motion):
        """Test isStopped() function."""
        assert not full_motion.isStopped()

    def test_isStoppedTrue(self, stopped_motion):
        """Test isStopped() function."""
        assert stopped_motion.isStopped()

    def test_hasSamePosTrue(self, full_motion):
        """Test the getTime() function."""
        m = Motion(
            1.0, [1.0000001, 2.0, 3.0], [1.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 1.0, 0.0], 2.0
        )
        assert full_motion.hasSamePos(motion=m, threshold=0.0001)

    def test_hasSamePosFalse(self, full_motion):
        """Test the getTime() function."""
        m = Motion(
            1.0, [1.01, 2.0, 3.0], [1.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 1.0, 0.0], 2.0
        )
        assert not full_motion.hasSamePos(motion=m, threshold=0.0001)

    def test_hasSameVelTrue(self, full_motion):
        """Test the getTime() function."""
        m = Motion(
            1.0, [2.0, 2.0, 3.0], [1.0000001, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 1.0, 0.0], 2.0
        )
        assert full_motion.hasSameVel(motion=m, threshold=0.0001)

    def test_hasSameVelFalse(self, full_motion):
        """Test the getTime() function."""
        m = Motion(
            1.0, [2.0, 2.0, 3.0], [1.01, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 1.0, 0.0], 2.0
        )
        assert not full_motion.hasSameVel(motion=m, threshold=0.0001)

    def test_hasSameAccTrue(self, full_motion):
        """Test the getTime() function."""
        m = Motion(
            1.0, [2.0, 2.0, 3.0], [0.0, 0.0, 0.0], [0.0, 0.0000001, 0.0], [0.0, 1.0, 0.0], 2.0
        )
        assert full_motion.hasSameAcc(motion=m, threshold=0.0001)

    def test_hasSameAccFalse(self, full_motion):
        """Test the getTime() function."""
        m = Motion(
            1.0, [2.0, 2.0, 3.0], [0.0, 0.0, 0.0], [0.0, 0.01, 0.0], [0.0, 1.0, 0.0], 2.0
        )
        assert not full_motion.hasSameAcc(motion=m, threshold=0.0001)

    def test_getTime(self, full_motion):
        """Test the getTime() function."""
        assert full_motion.getTime() == 1.0

    def test_getS(self, full_motion):
        """Test the getS() function."""
        assert full_motion.getS() == 2.0

    def test_getAngle(self, full_motion):
        """Test the getAngle() function."""
        assert full_motion.getAngle() == [1.0, 2.0, 3.0]

    def test_getVelocity(self, full_motion):
        """Test the getVelocity() function."""
        assert full_motion.getVelocity() == [1.0, 0.0, 0.0]

    def test_getAcceleration(self, full_motion):
        """Test the getAcceleration() function."""
        assert full_motion.getAcceleration() == [0.0, 0.0, 0.0]

    def test_getJerk(self, full_motion):
        """Test the getJerk() function."""
        assert full_motion.getJerk() == [0.0, 1.0, 0.0]

    def test_setTime(self, full_motion):
        """Test the setTime() function."""
        full_motion.setTime(2.0)
        assert full_motion.getTime() == 2.0

    def test_setS(self, full_motion):
        """Test the setS() function."""
        full_motion.setS(3.0)
        assert full_motion.getS() == 3.0

    def test_setAngle(self, full_motion):
        """Test the setAngle() function."""
        full_motion.setAngle([0.0, 0.0, 0.0])
        assert full_motion.getAngle() == [0.0, 0.0, 0.0]

    def test_setVelocity(self, full_motion):
        """Test the setVelocity() function."""
        full_motion.setVelocity([-1.0, -1.0, 1.0])
        assert full_motion.getVelocity() == [-1.0, -1.0, 1.0]

    def test_setAcceleration(self, full_motion):
        """Test the setAcceleration() function."""
        full_motion.setAcceleration([0.0, 0.0, 1.0])
        assert full_motion.getAcceleration() == [0.0, 0.0, 1.0]

    def test_setJerk(self, full_motion):
        """Test the setJerk() function."""
        full_motion.setJerk([0.0, 0.0, 0.0])
        assert full_motion.getJerk() == [0.0, 0.0, 0.0]
