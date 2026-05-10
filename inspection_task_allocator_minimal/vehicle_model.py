import math


class DifferentialDriveVehicle:
    def __init__(self, x, y, theta=0.0, v_max=0.6, omega_max=1.0, dt=0.1, radius=0.2):
        self.x = float(x)
        self.y = float(y)
        self.theta = float(theta)
        self.v_max = float(v_max)
        self.omega_max = float(omega_max)
        self.dt = float(dt)
        self.radius = float(radius)

    def _normalize_theta(self, theta):
        while theta > math.pi:
            theta -= 2.0 * math.pi
        while theta < -math.pi:
            theta += 2.0 * math.pi
        return theta

    def pose(self):
        return self.x, self.y, self.theta

    def position(self):
        return self.x, self.y

    def step(self, v, omega):
        v = max(0.0, min(self.v_max, float(v)))
        omega = max(-self.omega_max, min(self.omega_max, float(omega)))
        self.x += v * math.cos(self.theta) * self.dt
        self.y += v * math.sin(self.theta) * self.dt
        self.theta = self._normalize_theta(self.theta + omega * self.dt)

    def distance_to(self, point):
        px, py = point
        return math.hypot(self.x - float(px), self.y - float(py))

    def heading_to(self, point):
        px, py = point
        return math.atan2(float(py) - self.y, float(px) - self.x)

    def reset(self, x, y, theta=0.0):
        self.x = float(x)
        self.y = float(y)
        self.theta = self._normalize_theta(float(theta))
