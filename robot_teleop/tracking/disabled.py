from robot_teleop.registry import register


class DisabledTrackingAdapter:
    name = "disabled"

    def describe(self):
        return {"location": "none", "backend": "disabled", "optional": True}


register("tracking", DisabledTrackingAdapter.name, DisabledTrackingAdapter)
