"""Small logging helper for simulation events and timing."""

class EventLogger:
    def __init__(self):
        self.blue_time = None
        self.yellow_time = None
        self.last_state = None

    def state(self, sim_time: float, state_name: str):
        if state_name != self.last_state:
            print(f"[{sim_time:7.2f}s] STATE -> {state_name}")
            self.last_state = state_name

    def info(self, sim_time: float, message: str):
        print(f"[{sim_time:7.2f}s] {message}")

    def blue_reached(self, sim_time: float):
        if self.blue_time is None:
            self.blue_time = sim_time
            print(f"[{sim_time:7.2f}s] BLUE PILLAR REACHED")

    def yellow_reached(self, sim_time: float):
        if self.yellow_time is None:
            self.yellow_time = sim_time
            blue_to_yellow = None if self.blue_time is None else sim_time - self.blue_time
            print(f"[{sim_time:7.2f}s] YELLOW PILLAR REACHED")
            print("========================================")
            print("TIMING SUMMARY")
            print(f"Start -> Blue:   {self.blue_time:.2f} s" if self.blue_time is not None else "Start -> Blue: missing")
            print(f"Blue -> Yellow:  {blue_to_yellow:.2f} s" if blue_to_yellow is not None else "Blue -> Yellow: missing")
            print(f"Total:           {sim_time:.2f} s")
            print("========================================")
