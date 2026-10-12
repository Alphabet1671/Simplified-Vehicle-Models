from simplified_vehicle_models.powertrain.power_plant import compile_electric_motor, maximum_motor_torque
from simplified_vehicle_models.xml.loader import load_model

def test_motor_zero_speed_uses_first_row():
    p=compile_electric_motor(load_model("models/power-plants/amk_dd5_100c.xml"))
    assert maximum_motor_torque(p,0.0,600/3**0.5) > 0
    assert p.maximum_motor_speed == 18600
