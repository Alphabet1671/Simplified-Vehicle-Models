"""Reference TV controllers implementing the co-simulation protocol."""
from __future__ import annotations
import numpy as np
from simplified_vehicle_models.evolution.cosim import TvInputs

class PassthroughTV:
    """No vectoring: forward the driver's powertrain torques unchanged."""
    sample_time = 0.01
    def reset(self) -> None: pass
    def step(self, inputs: TvInputs) -> np.ndarray:
        return np.asarray(inputs.driver_torques, float).copy()
    def get_state(self): return None
    def set_state(self, state) -> None: pass
