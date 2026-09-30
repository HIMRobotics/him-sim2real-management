"""Measure motion and course completion independently of safety checks."""

import mujoco


class MotionMetrics:
    def __init__(self, plant):
        self.plant = plant
        self.start = plant.data.xpos[plant.base].copy()
        self.min_height = float(self.start[2])
        self.max_x = float(self.start[0])
        self.fell = False
        self.contacts = {
            name: 0
            for name in ("stair_1", "stair_2", "stair_3", "top_landing", "finish_pad")
        }
        self.platforms = {
            mujoco.mj_name2id(plant.model, mujoco.mjtObj.mjOBJ_GEOM, name): name
            for name in self.contacts
        }
        self.platforms.pop(-1, None)
        self.feet = {
            i
            for i in range(plant.model.ngeom)
            if "ankle_roll"
            in (plant.model.body(int(plant.model.geom_bodyid[i])).name or "")
            and plant.model.geom_contype[i] != 0
        }
        self.stood_on_top = False
        self.airborne_gap = False
        self.observe()

    def observe(self):
        plant = self.plant
        x, y, z = map(float, plant.data.xpos[plant.base])
        upright = float(plant.data.xmat[plant.base].reshape(3, 3)[2, 2])
        self.min_height = min(self.min_height, z)
        self.max_x = max(self.max_x, x)
        self.fell |= plant.pose == "standing" and (z < 0.28 or upright < 0.3)
        touching = set()
        for contact in plant.data.contact:
            first, second = map(int, contact.geom)
            for foot, surface in ((first, second), (second, first)):
                if foot in self.feet and surface in self.platforms:
                    name = self.platforms[surface]
                    self.contacts[name] += 1
                    touching.add(name)
        if (
            "top_landing" in touching
            and 1.4 <= x <= 2.05
            and abs(y) <= 0.45
            and z >= 0.55
            and upright >= 0.7
        ):
            self.stood_on_top = True
        if len(self.platforms) == 5 and 2.08 < x < 2.27 and not touching:
            heights = [
                plant.data.xpos[plant.model.body(side + "_ankle_roll_link").id, 2]
                for side in ("left", "right")
            ]
            if min(heights) > 0.13:
                self.airborne_gap = True

    def report(self):
        x, y, z = map(float, self.plant.data.xpos[self.plant.base])
        upright = float(self.plant.data.xmat[self.plant.base].reshape(3, 3)[2, 2])
        result = {
            "start_position": self.start.tolist(),
            "final_position": [x, y, z],
            "max_forward_position": self.max_x,
            "minimum_pelvis_height": self.min_height,
            "fell": bool(self.fell),
            "upright_at_end": upright >= 0.7,
        }
        if len(self.platforms) == 5:
            distinct = sum(
                self.contacts[name] > 0 for name in ("stair_1", "stair_2", "stair_3")
            )
            finish = (
                2.25 <= x <= 2.90
                and abs(y) <= 0.45
                and z >= 0.55
                and upright >= 0.7
                and self.contacts["finish_pad"] > 0
            )
            result["stairs"] = {
                "foot_contacts": dict(self.contacts),
                "distinct_treads": distinct,
                "stood_on_top": self.stood_on_top,
                "airborne_gap": self.airborne_gap,
                "finished_upright": finish,
                "climbed_stairs": bool(
                    distinct >= 2 and self.stood_on_top and not self.fell
                ),
                "course_success": bool(
                    distinct >= 2
                    and self.stood_on_top
                    and self.airborne_gap
                    and finish
                    and not self.fell
                ),
            }
        return result
