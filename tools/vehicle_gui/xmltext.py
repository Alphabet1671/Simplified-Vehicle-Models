"""Edit model XML as text, so comments, spacing and line order stay as they are.

``ElementTree`` cannot do this: it drops comments and rewrites ``<x />`` spacing, which
changes the bytes of 35 of the 62 files in ``models/``. Every function here changes only the
attribute it names, and never looks inside a comment.
"""
from __future__ import annotations

import math
import os
import re
from pathlib import Path

_COMMENT = re.compile(r"(<!--.*?-->)", re.S)
_PARAMETER = re.compile(r'(<parameter\b[^<>]*?\bname="(?P<name>[^"]+)"[^<>]*?\bvalue=")(?P<value>[^"]*)(")')
_UNIT = re.compile(r'<parameter\b[^<>]*?\bname="(?P<name>[^"]+)"[^<>]*?\bunit="(?P<unit>[^"]*)"')
_ROOT_ID = re.compile(r'(<model\b[^<>]*?\bid=")([^"]*)(")')
_REFERENCE_TAG = re.compile(r"<(?:component|source|data)\b[^<>]*>")
_ATTRIBUTE = re.compile(r'\b(ref|path)="([^"]*)"')
_COMPONENT = re.compile(r'<component\b[^<>]*\brole="(?P<role>[^"]+)"[^<>]*>')
_SOURCE = re.compile(r"<source\b[^<>]*>")


def _map_code(text: str, function) -> str:
    """Apply ``function`` to every stretch of text outside ``<!-- ... -->``."""
    return "".join(part if part.startswith("<!--") else function(part) for part in _COMMENT.split(text))


def parameter_values(text: str) -> dict[str, str]:
    """Return ``{name: value text}`` for every ``<parameter>`` outside comments."""
    found: dict[str, str] = {}

    def scan(code: str) -> str:
        for match in _PARAMETER.finditer(code):
            found[match.group("name")] = match.group("value")
        return code

    _map_code(text, scan)
    return found


def parameter_units(text: str) -> dict[str, str]:
    units: dict[str, str] = {}

    def scan(code: str) -> str:
        for match in _UNIT.finditer(code):
            units[match.group("name")] = match.group("unit")
        return code

    _map_code(text, scan)
    return units


def format_number(value: float) -> str:
    """Return the shortest text that reads back as exactly ``value``."""
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"a parameter value must be finite: {value}")
    return repr(value)


def set_parameters(text: str, values: dict[str, float]) -> str:
    """Set ``value=`` of the named parameters. Unknown names raise ``ValueError``."""
    pending = {name: format_number(value) for name, value in values.items()}

    def edit(code: str) -> str:
        def one(match):
            name = match.group("name")
            if name in pending:
                return match.group(1) + pending.pop(name) + match.group(4)
            return match.group(0)

        return _PARAMETER.sub(one, code)

    result = _map_code(text, edit)
    if pending:
        raise ValueError(f"no such parameter: {', '.join(sorted(pending))}")
    return result


def model_id(text: str) -> str:
    match = _ROOT_ID.search(_COMMENT.sub("", text))
    if match is None:
        raise ValueError("the file has no model id")
    return match.group(2)


def set_model_id(text: str, new_id: str) -> str:
    """Set the root ``id``. The new id must not contain a quote or an angle bracket."""
    if not new_id or any(character in new_id for character in '"<>&\n'):
        raise ValueError(f"not a usable model id: {new_id!r}")
    done: list[int] = []

    def edit(code: str) -> str:
        if done:
            return code

        def one(match):
            done.append(1)
            return match.group(1) + new_id + match.group(3)

        return _ROOT_ID.sub(one, code, count=1)

    result = _map_code(text, edit)
    if not done:
        raise ValueError("the file has no model id")
    return result


def _set_ref(tag: str, ref: str) -> str:
    return _ATTRIBUTE.sub(lambda a: f'ref="{ref}"' if a.group(1) == "ref" else a.group(0), tag, count=0)


def _check_ref(ref: str) -> str:
    if not ref or any(character in ref for character in '"<>&\n'):
        raise ValueError(f"not a usable reference: {ref!r}")
    return ref


def set_component_ref(text: str, role: str, ref: str) -> str:
    """Point the ``<component role=...>`` at ``ref``."""
    _check_ref(ref)
    done: list[int] = []

    def edit(code: str) -> str:
        def one(match):
            if match.group("role") != role:
                return match.group(0)
            done.append(1)
            return _set_ref(match.group(0), ref)

        return _COMPONENT.sub(one, code)

    result = _map_code(text, edit)
    if not done:
        raise ValueError(f"no component with role {role!r}")
    return result


def set_source_ref(text: str, ref: str) -> str:
    """Point the first ``<source>`` of a formulation at ``ref``."""
    _check_ref(ref)
    done: list[int] = []

    def edit(code: str) -> str:
        def one(match):
            if done:
                return match.group(0)
            done.append(1)
            return _set_ref(match.group(0), ref)

        return _SOURCE.sub(one, code)

    result = _map_code(text, edit)
    if not done:
        raise ValueError("the file has no source element")
    return result


def reanchor(text: str, source_directory: Path, target_directory: Path) -> str:
    """Rewrite relative ``ref`` and ``path`` attributes so they still work from ``target_directory``."""
    source_directory, target_directory = Path(source_directory).resolve(), Path(target_directory).resolve()
    if source_directory == target_directory:
        return text

    def fix(match):
        value = match.group(2)
        if Path(value).is_absolute():
            return match.group(0)
        target = (source_directory / value).resolve()
        return f'{match.group(1)}="{Path(os.path.relpath(target, target_directory)).as_posix()}"'

    return _map_code(text, lambda code: _REFERENCE_TAG.sub(lambda tag: _ATTRIBUTE.sub(fix, tag.group(0)), code))
