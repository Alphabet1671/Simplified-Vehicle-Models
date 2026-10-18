"""Vehicle assembly configuration and nested component selection."""
from __future__ import annotations

from dataclasses import dataclass
import importlib
from pathlib import Path
from xml.etree import ElementTree

from .catalogue import fits, read_entry


@dataclass(frozen=True, slots=True)
class Slot:
    key: str
    label: str
    parent: str | None


@dataclass(frozen=True, slots=True)
class VehicleConfiguration:
    source: Path
    formulation: Path | None
    assembly: Path
    model: str
    choices: dict[str, Path]

    @classmethod
    def open(cls, path: Path) -> "VehicleConfiguration":
        source = Path(path).resolve()
        try:
            root = ElementTree.parse(source).getroot()
        except (OSError, ElementTree.ParseError) as error:
            raise ValueError(f"cannot read {source}: {error}") from error
        if root.tag != "model":
            raise ValueError(f"{source}: the root element must be model")
        if root.get("type") == "vehicle-formulation":
            sources = root.findall("./source")
            if len(sources) != 1 or not sources[0].get("ref"):
                raise ValueError(f"{source}: formulation must have exactly one source ref")
            formulation = source
            assembly = _resolve(source, sources[0].get("ref"))
            model = root.get("subtype", "")
        elif root.get("type") == "vehicle-assembly":
            formulation = None
            assembly = source
            model = "planar-7dof"
        else:
            raise ValueError(f"{source}: expected vehicle-formulation or vehicle-assembly")
        choices = _read_choices(assembly)
        return cls(source, formulation, assembly, model, choices)

    def slots(self) -> tuple[Slot, ...]:
        result: list[Slot] = []
        _append_slots(self.choices, "", None, result, set())
        return tuple(result)

    @property
    def runnable(self) -> bool:
        runner = importlib.import_module("tools.run.run_iem24_forward_driver")
        return self.model in runner.MODELS

    def with_choice(self, key: str, path: Path) -> "VehicleConfiguration":
        path = Path(path).resolve()
        current = self.choices.get(key)
        if current is None:
            raise ValueError(f"unknown slot {key}")
        if not fits(read_entry(current), read_entry(path)):
            raise ValueError(f"component does not fit slot {key}")
        choices = dict(self.choices)
        choices[key] = path
        prefix = key + "/"
        for descendant in tuple(choices):
            if descendant.startswith(prefix):
                del choices[descendant]
        _append_choice_descendants(choices, key, path, set())
        return VehicleConfiguration(self.source, self.formulation, self.assembly, self.model, choices)

    def with_model(self, name: str) -> "VehicleConfiguration":
        runner = importlib.import_module("tools.run.run_iem24_forward_driver")
        if name not in runner.MODELS:
            raise ValueError(f"unknown model {name}")
        return VehicleConfiguration(self.source, self.formulation, self.assembly, name, dict(self.choices))

    def changed_from(self, baseline: "VehicleConfiguration") -> tuple[str, ...]:
        keys = [slot.key for slot in self.slots()]
        changed = [key for key in keys if self.choices.get(key) != baseline.choices.get(key)]
        if self.model != baseline.model:
            changed.append("model")
        return tuple(dict.fromkeys(changed))

    def parameters(self, key: str) -> list[tuple[str, float, str]]:
        path = self.choices.get(key)
        if path is None:
            raise ValueError(f"unknown slot {key}")
        try:
            root = ElementTree.parse(path).getroot()
        except (OSError, ElementTree.ParseError) as error:
            raise ValueError(f"cannot read {path}: {error}") from error
        result = []
        for parameter in root.findall("./parameters/parameter"):
            try:
                value = float(parameter.get("value", ""))
            except ValueError as error:
                raise ValueError(f"{path}: parameter has non-numeric value") from error
            result.append((parameter.get("name", ""), value, parameter.get("unit", "")))
        return result


def _resolve(parent: Path, ref: str | None) -> Path:
    if not ref:
        raise ValueError(f"{parent}: component has no ref")
    return (parent.parent / ref).resolve()


def _components(path: Path):
    try:
        root = ElementTree.parse(path).getroot()
    except (OSError, ElementTree.ParseError) as error:
        raise ValueError(f"cannot read {path}: {error}") from error
    return root.findall("./components/component")


def _read_choices(assembly: Path) -> dict[str, Path]:
    choices: dict[str, Path] = {}
    _collect_choices(assembly, "", choices, set())
    return choices


def _collect_choices(path: Path, prefix: str, choices: dict[str, Path], branch: set[Path]) -> None:
    path = path.resolve()
    if path in branch:
        raise ValueError(f"component cycle at {path}")
    next_branch = branch | {path}
    for component in _components(path):
        role = component.get("role")
        if not role:
            raise ValueError(f"{path}: component has no role")
        key = f"{prefix}/{role}" if prefix else role
        child = _resolve(path, component.get("ref"))
        choices[key] = child
        _collect_choices(child, key, choices, next_branch)


def _append_choice_descendants(choices: dict[str, Path], key: str, path: Path, branch: set[Path]) -> None:
    path = path.resolve()
    if path in branch:
        raise ValueError(f"component cycle at {path}")
    next_branch = branch | {path}
    for component in _components(path):
        role = component.get("role")
        if not role:
            raise ValueError(f"{path}: component has no role")
        child_key = f"{key}/{role}"
        child = _resolve(path, component.get("ref"))
        choices[child_key] = child
        _append_choice_descendants(choices, child_key, child, next_branch)


def _append_slots(choices: dict[str, Path], prefix: str, parent: str | None,
                  result: list[Slot], branch: set[Path]) -> None:
    for key, path in choices.items():
        depth = key.count("/") + 1
        if (prefix == "" and depth != 1) or (prefix and (not key.startswith(prefix + "/") or key.count("/") != prefix.count("/") + 1)):
            continue
        role = key.rsplit("/", 1)[-1]
        slot_parent = prefix or None
        result.append(Slot(key, " › ".join(key.split("/")), slot_parent))
        path = path.resolve()
        if path in branch:
            raise ValueError(f"component cycle at {path}")
        if _components(path):
            _append_slots(choices, key, key, result, branch | {path})
