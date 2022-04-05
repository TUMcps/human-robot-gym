import math
from scipy.spatial.transform import Rotation

import numpy as np


class PlotCapsule:
    """Capsule with different formats for plotting."""
    def __init__(self, p1, p2, r):
      """Initialize a plot capsule.
      Args:
        p1: Point 1 (x, y, z)
        p2: Point 2 (x, y, z)
        r: Radius
      """
      self.update_pos(p1, p2, r)

    def update_pos(self, p1, p2, r):
      """Update the position (and possibly radius) of the capsule
      """
      self.p1 = p1
      self.p2 = p2 
      self.r = r
      # Convert to pos, size, mat format.
      p1x = p1[0]
      p1y = p1[1]
      p1z = p1[2]
      p2x = p2[0]
      p2y = p2[1]
      p2z = p2[2]
      v2_x = (p2x-p1x)
      v2_y = (p2y-p1y)
      v2_z = (p2z-p1z)
      norm = math.sqrt(math.pow(v2_x, 2) + math.pow(v2_y, 2) + math.pow(v2_z, 2))
      # POS
      self.pos = [(p1x + p2x)/2, (p1y + p2y)/2, (p1z + p2z)/2]
      if norm > 1e-6:
        # SIZE
        self.size = [r, r, norm/2]
        # ORIENTATION
        # Rotate z axis vector to direction vector according to https://stackoverflow.com/questions/1171849/finding-quaternion-representing-the-rotation-from-one-vector-to-another/1171995#1171995
        a_x = -v2_y/norm
        a_y = v2_x/norm
        a_z = 0
        a_w = 1 + v2_z/norm
        norm_q = math.sqrt(math.pow(a_w, 2) + math.pow(a_x, 2) + math.pow(a_y, 2) + math.pow(a_z, 2))
        w = a_w/norm_q
        x = a_x/norm_q
        y = a_y/norm_q
        z = a_z/norm_q
        rot = Rotation.from_quat([x, y, z, w])
        self.mat = np.array(rot.as_matrix())
      else:
        self.size = [r, r, 0.00001]
        self.mat = np.eye(3)


