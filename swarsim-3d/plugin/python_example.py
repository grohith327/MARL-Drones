from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import collections
import math
import numpy as np
import scipy as sp
import scipy.interpolate
import scipy.optimize
import swarmsim
import swarmui
import random

# PyOpenGL can be used to draw on the simulation screen.
from OpenGL.GL import *
from OpenGL.GLU import *

_ARENA_SIZE = 3.


class Supervisor(swarmsim.Supervisor):

  def initialize(self, arg_string):
    if not arg_string:
      arg_string = '2,2,2'
    num_unicycles, num_quadrotors, num_bicycles = [int(v) for v in arg_string.split(',')]

    self._robots = []

    for i in range(num_bicycles):
      x = random.random() * _ARENA_SIZE - _ARENA_SIZE / 2.
      y = random.random() * _ARENA_SIZE - _ARENA_SIZE / 2.
      a = random.random() * 2. * math.pi
      self._robots.append(MyBicycle(x, y, a, i))

    for i in range(num_unicycles):
      x = random.random() * _ARENA_SIZE - _ARENA_SIZE / 2.
      y = random.random() * _ARENA_SIZE - _ARENA_SIZE / 2.
      a = random.random() * 2. * math.pi
      self._robots.append(MyUnicycle(0, .20, 0., i + num_bicycles))

    for i in range(num_quadrotors):
      x = random.random() * _ARENA_SIZE - _ARENA_SIZE / 2.
      y = random.random() * _ARENA_SIZE - _ARENA_SIZE / 2.
      z = random.random() * _ARENA_SIZE / 2.
      a = random.random() * 2. * math.pi
      self._robots.append(MyQuadrotor(x, y, z, a, 0., 0., i + num_unicycles + num_bicycles))

    return self._robots

  def execute(self, t, dt):
    # We can modify the state of the different robots in this function.
    # E.g.: self._robots[0].x = 0.
    pass

  def draw(self, t):
    # We can draw things in OpenGL.
    # PyOpenGL generally works but has issues for some functions.
    # Use swarmui for the problematic functions. E.g.,
    swarmui.glLineWidth(2)
    swarmui.glBegin(GL_LINES)
    swarmui.glColor3f(0.4, 0., 0.)
    glVertex3f(0., .0, 0.)
    glVertex3f(0., .5, 0.)
    swarmui.glEnd()
    for r in self._robots:
      r.draw(t)
    pass


########################################
# Example of differential-wheel robot. #
########################################


class MyUnicycle(swarmsim.Unicycle):

  def initialize(self):
    print('initialize():', self)

  def execute(self, t, dt):
    return swarmsim.UnicycleControls(u=0.2, w=0.5)


#########################################
# Example of car driving along a track. #
#########################################


_RADIUS = 1.
_STRAIGHT = 2.
_NUM_POINTS = 10
_WHEELBASE = .12
_SIDE = .05
_L2 = _WHEELBASE * .5


Reference = collections.namedtuple('Reference', ['x', 'y', 'yaw', 'phi', 'warm_start'])


class Spline(object):

  def __init__(self, points):
    points = np.array(points, dtype=np.float32)
    v = .3  # m/s.
    dt = np.linalg.norm(points[:-1] - points[1:], axis=1) / v
    last_dt = np.linalg.norm(points[0] - points[-1]) / v
    t = np.cumsum([0] + dt.tolist() + [last_dt])
    points = np.append(points, np.expand_dims(points[0, :], 0), axis=0)
    self._x = sp.interpolate.CubicSpline(t, points[:, 0], bc_type='periodic')
    self._y = sp.interpolate.CubicSpline(t, points[:, 1], bc_type='periodic')
    self._ts = t[:-1]
    self._points = points[:-1]

  def _closest(self, x, y, last_t=None):
    p = np.array([x, y], dtype=np.float32)

    # Global search.
    if last_t is None:
      ds = np.linalg.norm(self._points - np.expand_dims(p, 0), axis=1)
      idx = np.argmin(ds)
      last_t = self._ts[idx]

    # Local neighborhood search.
    def dist(t):
      d = np.array([self._x(t)[0], self._y(t)[0]], dtype=np.float32)
      return np.linalg.norm(d - p)
    return sp.optimize.fmin(dist, last_t, disp=False).item()

  def reference(self, x, y, last_t=None):
    t = self._closest(x, y)
    x = self._x(t)
    y = self._y(t)
    xdot = self._x(t, 1)
    ydot = self._y(t, 1)
    a = np.arctan2(ydot, xdot)
    xdot2 = self._x(t, 2)
    ydot2 = self._y(t, 2)
    c = xdot * ydot2 - ydot * xdot2 / np.power(xdot ** 2 + ydot ** 2, 3. / 2.)
    phi = np.arctan(_WHEELBASE * c)
    return Reference(x, y, a, phi, t)


class MyBicycle(swarmsim.Bicycle):

  def initialize(self):
    print('initialize():', self)
    # Half circle, followed by straight and then loop.
    self._keypoints = []
    # Half circle.
    a = np.linspace(0., np.pi, _NUM_POINTS)[1:]
    self._keypoints.extend(zip(np.cos(a) * _RADIUS,
                               np.sin(a) * _RADIUS))
    # Straight.
    d = np.linspace(0., _STRAIGHT, _NUM_POINTS)[1:]
    self._keypoints.extend(zip(np.zeros_like(d) - _RADIUS, -d))
    # Half circle.
    a = np.linspace(np.pi, 2. * np.pi, _NUM_POINTS)[1:]
    self._keypoints.extend(zip(np.cos(a) * _RADIUS,
                               np.sin(a) * _RADIUS - _STRAIGHT))
    # Straight.
    d = np.linspace(0., _STRAIGHT, _NUM_POINTS)[:-1][::-1]
    self._keypoints.extend(zip(np.zeros_like(d) + _RADIUS, -d))
    self._trajectory = Spline(self._keypoints)
    self._desired = self._trajectory.reference(self.x, self.y)

  def to_local(self, reference):
    xy = np.array([[reference.x - self.x], [reference.y - self.y]], dtype=np.float32)
    c = np.cos(self.yaw)
    s = np.sin(self.yaw)
    R = np.array([[c, s], [-s, c]], dtype=np.float32)
    xy = R.dot(xy)
    return Reference(xy[0], xy[1], reference.yaw - self.yaw, reference.phi, reference.warm_start)

  def execute(self, t, dt):
    self._desired = self._trajectory.reference(self.x, self.y, self._desired.warm_start)
    local = self.to_local(self._desired)
    x = local.x + _WHEELBASE * np.cos(local.yaw) + _L2 * np.cos(local.yaw + local.phi)
    y = local.y + _WHEELBASE * np.sin(local.yaw) + _L2 * np.sin(local.yaw + local.phi)
    phi = np.arctan2(y, x - _WHEELBASE)
    return swarmsim.BicycleControls(v=0.5, phi=phi)

  def draw(self, t):
    if self.identifier == 0:
      swarmui.glBegin(GL_LINE_LOOP)
      swarmui.glColor3f(0., 0., .4)
      for p in self._keypoints:
        glVertex3f(p[0], .01, -p[1])
      swarmui.glEnd()
    # Reference vehicle.
    swarmui.glColor3f(4., 0., 0.)
    swarmui.glBegin(GL_LINES)
    fx = self._desired.x + np.cos(self._desired.yaw) * _WHEELBASE
    fy = self._desired.y + np.sin(self._desired.yaw) * _WHEELBASE
    glVertex3f(self._desired.x, .02, -self._desired.y)  # Wheelbase.
    glVertex3f(fx, .02, -fy)
    sx = np.sin(self._desired.yaw) * _SIDE
    sy = -np.cos(self._desired.yaw) * _SIDE
    glVertex3f(self._desired.x + sx, .02, -self._desired.y - sy)  # Rear axle.
    glVertex3f(self._desired.x - sx, .02, -self._desired.y + sy)
    glVertex3f(fx + sx, .02, -fy - sy)  # Front axle.
    glVertex3f(fx - sx, .02, -fy + sy)
    glVertex3f(fx, .02, -fy)  # Steering.
    glVertex3f(fx + np.cos(self._desired.yaw + self._desired.phi) * _L2, .02,
               -fy - np.sin(self._desired.yaw + self._desired.phi) * _L2)
    swarmui.glEnd()


#################################################
# Example of quadrotor reaching a desired goal. #
#################################################


_MAX_GOAL_DISTANCE = 1.2
_GRAVITY = 9.81
_KP = 1.
_KV = 2.
_KR = 10.
_KW = 5.
_KR_YAW = .1
_KW_YAW = .3
_MASS = .5
_EPS = 1e-3


class MyQuadrotor(swarmsim.Quadrotor):

  def initialize(self):
    print('initialize():', self)
    self._goal = np.array([0., 0., self.identifier * .2], dtype=np.float32)

  def execute(self, t, dt):
    return self.control_to_goal(self._goal)

  def control_to_goal(self, goal):
    current_position = np.array([self.x, self.y, self.z], dtype=np.float32)
    current_angles = np.array([self.roll, self.pitch, self.yaw], dtype=np.float32)
    current_speed = np.array([self.xdot, self.ydot, self.zdot], dtype=np.float32)
    gravity = np.array([0., 0., _GRAVITY * _MASS], dtype=np.float32)

    desired_position = goal - current_position
    desired_yaw = self.yaw

    # Desired force.
    norm = np.linalg.norm(desired_position)
    if norm > _MAX_GOAL_DISTANCE:
      desired_position /= norm
    desired_force = _KP * desired_position - _KV * current_speed + gravity

    # Body frame (R).
    c = np.cos(current_angles)
    s = np.sin(current_angles)
    body_frame = np.array([
        [c[1] * c[2], c[2] * s[0] * s[1] - c[0] * s[2],  c[0] * c[2] * s[1] + s[0] * s[2]],
        [c[1] * s[2], c[0] * c[2] + s[0] * s[1] * s[2], -c[2] * s[0] + c[0] * s[1] * s[2]],
        [-s[1], c[1] * s[0], c[0] * c[1]]], dtype=np.float32)

    # Goal frame. We really only need the x-axis.
    x_axis_goal_frame = np.array([np.cos(desired_yaw), np.sin(desired_yaw), 0.], dtype=np.float32)

    # We can already compute the up force.
    up_force = desired_force.dot(body_frame[:, 2])

    desired_force_norm = np.linalg.norm(desired_force)
    if desired_force_norm < _EPS:
      return swarmsim.QuadrotorControls(up_force=up_force, roll_torque=0., pitch_torque=0., yaw_torque=0.)

    # Desired axes (R_d).
    desired_z = desired_force / desired_force_norm
    desired_y = np.cross(desired_z, x_axis_goal_frame)
    desired_y_norm = np.linalg.norm(desired_y)
    desired_y /= desired_y_norm  # Renormalize for numerical accuracy.
    desired_x = np.cross(desired_y, desired_z)
    desired_frame = np.array([desired_x, desired_y, desired_z]).T

    # R^T * R_d
    T = body_frame.T.dot(desired_frame)

    # (R_d^T * R - R^T * R_d) * 0.5
    eR = (T.T - T) * 0.5
    roll_torque = _KR * eR[1, 2] - _KW * self.rolldot
    pitch_torque = - _KR * eR[0, 2] - _KW * self.pitchdot
    yaw_torque = _KR_YAW * eR[0, 1] - _KW_YAW * self.yawdot

    return swarmsim.QuadrotorControls(up_force, roll_torque, pitch_torque, yaw_torque)
