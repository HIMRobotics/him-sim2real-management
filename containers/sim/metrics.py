"""Measure motion independently of safety checks."""


class MotionMetrics:
    def __init__(self, plant):
        self.plant = plant
        self.start = plant.data.xpos[plant.base].copy()
        self.min_height = float(self.start[2])
        self.max_x = float(self.start[0])
        self.fell = False
        self.observe()

    def observe(self):
        plant = self.plant
        x, _y, z = map(float, plant.data.xpos[plant.base])
        upright = float(plant.data.xmat[plant.base].reshape(3, 3)[2, 2])
        self.min_height = min(self.min_height, z)
        self.max_x = max(self.max_x, x)
        self.fell |= plant.pose == "standing" and (z < 0.28 or upright < 0.3)

    def report(self):
        x, y, z = map(float, self.plant.data.xpos[self.plant.base])
        upright = float(self.plant.data.xmat[self.plant.base].reshape(3, 3)[2, 2])
        return {
            "start_position": self.start.tolist(),
            "final_position": [x, y, z],
            "max_forward_position": self.max_x,
            "minimum_pelvis_height": self.min_height,
            "fell": bool(self.fell),
            "upright_at_end": upright >= 0.7,
        }
