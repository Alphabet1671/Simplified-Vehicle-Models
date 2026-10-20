"""Edit model XML as text while preserving comments, spacing and line order."""
from __future__ import annotations

import math
import os
import re
from pathlib import Path

_COMMENT = re.compile(r"(<!--.*?-->)", re.S)
_TAG = re.compile(r"<(parameter|component|source|data|model)\b[^<>]*>")
_ATTR = re.compile(r"(?P<prefix>\b(?P<name>[A-Za-z_:][\w:.-]*)\s*=\s*)(?P<quote>[\"'])(?P<value>.*?)(?P=quote)")


def _map_code(text: str, function) -> str:
    return "".join(part if part.startswith("<!--") else function(part) for part in _COMMENT.split(text))


def _tag_attrs(tag: str) -> dict[str, str]:
    return {m.group("name"): m.group("value") for m in _ATTR.finditer(tag)}


def _replace_attr(tag: str, name: str, value: str) -> tuple[str, bool]:
    def replace(match: re.Match[str]) -> str:
        if match.group("name") != name:
            return match.group(0)
        return match.group("prefix") + match.group("quote") + value + match.group("quote")
    changed = False
    def replace_checked(match: re.Match[str]) -> str:
        nonlocal changed
        result = replace(match)
        if match.group("name") == name:
            changed = True
        return result
    result = _ATTR.sub(replace_checked, tag)
    return result, changed


def parameter_values(text: str) -> dict[str, str]:
    found: dict[str, str] = {}
    def scan(code: str) -> str:
        for match in _TAG.finditer(code):
            if match.group(1) == "parameter":
                attrs = _tag_attrs(match.group(0))
                if "name" in attrs and "value" in attrs:
                    found[attrs["name"]] = attrs["value"]
        return code
    _map_code(text, scan)
    return found


def parameter_units(text: str) -> dict[str, str]:
    units: dict[str, str] = {}
    def scan(code: str) -> str:
        for match in _TAG.finditer(code):
            if match.group(1) == "parameter":
                attrs = _tag_attrs(match.group(0))
                if "name" in attrs and "unit" in attrs:
                    units[attrs["name"]] = attrs["unit"]
        return code
    _map_code(text, scan)
    return units


def format_number(value: float) -> str:
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"a parameter value must be finite: {value}")
    return repr(value)


def set_parameters(text: str, values: dict[str, float]) -> str:
    pending = {name: format_number(value) for name, value in values.items()}
    def edit(code: str) -> str:
        def one(match: re.Match[str]) -> str:
            tag = match.group(0)
            attrs = _tag_attrs(tag)
            if match.group(1) != "parameter" or attrs.get("name") not in pending:
                return tag
            result, changed = _replace_attr(tag, "value", pending.pop(attrs["name"]))
            return result if changed else tag
        return _TAG.sub(one, code)
    result = _map_code(text, edit)
    if pending:
        raise ValueError(f"no such parameter: {', '.join(sorted(pending))}")
    return result


def _root_model_span(text: str) -> tuple[int, int] | None:
    masked = _COMMENT.sub(lambda m: " " * len(m.group(0)), text)
    prefix = re.compile(r"\s*(?:<\?xml\b[^>]*\?>\s*)?")
    prefix_match = prefix.match(masked)
    start = prefix_match.end() if prefix_match else 0
    match = re.match(r"<model\b[^<>]*>", masked[start:])
    if match is None:
        return None
    return start + match.start(), start + match.end()


def _root_model_match(text: str) -> re.Match[str] | None:
    span = _root_model_span(text)
    if span is None:
        return None
    return re.compile(r"<model\b[^<>]*>").match(text, span[0], span[1])


def model_id(text: str) -> str:
    match = _root_model_match(text)
    if match is None:
        raise ValueError("the file has no model id")
    attrs = _tag_attrs(match.group(0))
    if "id" not in attrs:
        raise ValueError("the file has no model id")
    return attrs["id"]


def set_model_id(text: str, new_id: str) -> str:
    if not new_id or any(c in new_id for c in '"\'<>&\n'):
        raise ValueError(f"not a usable model id: {new_id!r}")
    span = _root_model_span(text)
    if span is None:
        raise ValueError("the file has no model id")
    start, end = span
    tag = text[start:end + 1]
    result, changed = _replace_attr(tag, "id", new_id)
    if not changed:
        raise ValueError("the file has no model id")
    return text[:start] + result + text[end + 1:]


def _check_ref(ref: str) -> str:
    if not ref or any(c in ref for c in '"\'<>&\n'):
        raise ValueError(f"not a usable reference: {ref!r}")
    return ref


def set_component_ref(text: str, role: str, ref: str) -> str:
    _check_ref(ref)
    done = False
    def edit(code: str) -> str:
        nonlocal done
        def one(match: re.Match[str]) -> str:
            nonlocal done
            tag = match.group(0)
            if match.group(1) != "component" or _tag_attrs(tag).get("role") != role:
                return tag
            result, changed = _replace_attr(tag, "ref", ref)
            if not changed:
                raise ValueError(f"component with role {role!r} has no ref attribute")
            done = True
            return result
        return _TAG.sub(one, code)
    result = _map_code(text, edit)
    if not done:
        raise ValueError(f"no component with role {role!r}")
    return result


def set_source_ref(text: str, ref: str) -> str:
    _check_ref(ref)
    done = False
    def edit(code: str) -> str:
        nonlocal done
        def one(match: re.Match[str]) -> str:
            nonlocal done
            tag = match.group(0)
            if match.group(1) != "source" or done:
                return tag
            result, changed = _replace_attr(tag, "ref", ref)
            if not changed:
                raise ValueError("source element has no ref attribute")
            done = True
            return result
        return _TAG.sub(one, code)
    result = _map_code(text, edit)
    if not done:
        raise ValueError("the file has no source element")
    return result


def reanchor(text: str, source_directory: Path, target_directory: Path) -> str:
    source_directory, target_directory = Path(source_directory).resolve(), Path(target_directory).resolve()
    if source_directory == target_directory:
        return text
    def edit(code: str) -> str:
        def one(match: re.Match[str]) -> str:
            tag = match.group(0)
            if match.group(1) not in {"component", "source", "data"}:
                return tag
            def fix(attr: re.Match[str]) -> str:
                if attr.group("name") not in {"ref", "path"}:
                    return attr.group(0)
                value = attr.group("value")
                if Path(value).is_absolute():
                    return attr.group(0)
                target = (source_directory / value).resolve()
                new = Path(os.path.relpath(target, target_directory)).as_posix()
                return attr.group("prefix") + attr.group("quote") + new + attr.group("quote")
            return _ATTR.sub(fix, tag)
        return _TAG.sub(one, code)
    return _map_code(text, edit)
