from simplified_vehicle_models.powertrain.energy_storage import compile_battery_pack, evaluate_storage, storage_state
from simplified_vehicle_models.xml.loader import load_model

def test_battery_pack_limits():
    p=compile_battery_pack(load_model("models/energy-storages/murphy_2024_battery.xml"))
    result=evaluate_storage(p,storage_state(1.0))
    assert result.bus_voltage > 500
    assert result.discharge_power_limit > 0
    assert result.usable_energy > 0
