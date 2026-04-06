import time

class SnakeSearch:
    """
    Lawnmower / Snake Search Pattern for field scanning.
    Moves in parallel rows covering the field.
    """
    def __init__(self, row_length=15.0, row_spacing=3.0, speed=0.4):
        self.row_length = row_length # Time to fly one row (seconds)
        self.row_spacing = row_spacing # Time to side-step (seconds)
        self.speed = speed
        
        self.start_time = time.time()
        self.state = "ROW_A" # ROW_A (Forward), SIDE_A (Right), ROW_B (Backward), SIDE_B (Right)
        self.state_start = time.time()
        
    def update(self):
        """Processes state transitions based on elapsed time."""
        elapsed = time.time() - self.state_start
        
        if self.state == "ROW_A":
            if elapsed > self.row_length:
                self._next_state("SIDE_A")
        elif self.state == "SIDE_A":
            if elapsed > self.row_spacing:
                self._next_state("ROW_B")
        elif self.state == "ROW_B":
            if elapsed > self.row_length:
                self._next_state("SIDE_B")
        elif self.state == "SIDE_B":
            if elapsed > self.row_spacing:
                self._next_state("ROW_A")

    def get_velocity(self):
        """
        Returns (vx, vy, vz) for the current state.
        Webots Drone: +vx is Forward, +vy is Left.
        """
        if self.state == "ROW_A": # Forward
            return self.speed, 0.0, 0.0
        elif self.state == "SIDE_A": # Side step (Right = -vy)
            return 0.0, -self.speed, 0.0
        elif self.state == "ROW_B": # Backward
            return -self.speed, 0.0, 0.0
        elif self.state == "SIDE_B": # Side step (Right = -vy)
            return 0.0, -self.speed, 0.0
            
        return 0.0, 0.0, 0.0

    def _next_state(self, new_state):
        self.state = new_state
        self.state_start = time.time()

    def reset(self):
        self.state = "ROW_A"
        self.state_start = time.time()
