from robot_teleop.registry import register


class BrowserMediaPipeTrackingAdapter:
    name = "browser_mediapipe"

    def describe(self):
        return {"location": "browser", "backend": "MediaPipe", "optional": True}


register("tracking", BrowserMediaPipeTrackingAdapter.name, BrowserMediaPipeTrackingAdapter)
