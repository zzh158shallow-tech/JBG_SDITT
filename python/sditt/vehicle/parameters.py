from __future__ import annotations

import ast
import operator
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np


ASSIGNMENT_RE = re.compile(r"Par_Vehicle\.(?P<name>\w+)(?P<index>\([^=]+\))?\s*=\s*(?P<expr>.+)")


@dataclass(frozen=True)
class VehicleParameters:
    """Raw vehicle parameters parsed from a MATLAB parameter script."""

    path: Path
    values: dict[str, Any]
    unsupported_statements: tuple[str, ...]


def load_vehicle_parameters(path: str | Path, *, vlc: float = 350 / 3.6) -> VehicleParameters:
    """Parse direct ``Par_Vehicle`` assignments from a MATLAB script.

    This is intentionally a reader, not a MATLAB interpreter. It captures scalar,
    vector, matrix, and simple indexed assignments that define CRH380A parameter
    data. Statements that depend on MATLAB functions such as ``sortrows`` are
    reported in ``unsupported_statements`` for later migration work.
    """

    file_path = Path(path)
    statements = _collect_statements(file_path.read_text(encoding="utf-8", errors="ignore"))
    values: dict[str, Any] = {}
    unsupported: list[str] = []
    context = {"InpPar.Vlc": vlc}

    for statement in statements:
        match = ASSIGNMENT_RE.match(statement)
        if not match:
            continue
        name = match.group("name")
        index = match.group("index")
        expression = match.group("expr").strip()

        try:
            value = _parse_value(expression, values, context)
        except (SyntaxError, ValueError):
            unsupported.append(statement)
            continue

        if index:
            try:
                values[name] = _assign_indexed(values.get(name), index, value)
            except ValueError:
                unsupported.append(statement)
        else:
            values[name] = value

    if "K_STy_Table" in values:
        values["K_STy_Table"] = _mirror_and_sort_lateral_stop_table(values["K_STy_Table"])
        unsupported = [
            statement
            for statement in unsupported
            if "K_STy_Table" not in statement
        ]

    return VehicleParameters(
        path=file_path,
        values=values,
        unsupported_statements=tuple(unsupported),
    )


def _collect_statements(text: str) -> list[str]:
    statements: list[str] = []
    buffer = ""
    for raw_line in text.splitlines():
        line = raw_line.split("%", 1)[0].strip()
        if not line:
            continue
        if line.startswith("function "):
            continue
        if line.endswith("..."):
            buffer += line[:-3] + " "
            continue
        buffer += line
        if ";" in line:
            parts = buffer.split(";")
            statements.extend(part.strip() for part in parts[:-1] if part.strip())
            buffer = parts[-1].strip()
        else:
            buffer += " "
    if buffer.strip():
        statements.append(buffer.strip())
    return statements


def _parse_value(expression: str, values: dict[str, Any], context: dict[str, float]) -> Any:
    expression = expression.strip()
    if expression.startswith("[") and expression.endswith("]"):
        return _parse_matrix(expression, values, context)
    return _eval_scalar(expression, values, context)


def _parse_matrix(expression: str, values: dict[str, Any], context: dict[str, float]) -> np.ndarray:
    inner = expression[1:-1].strip()
    if not inner:
        return np.empty((0, 0), dtype=float)

    rows: list[list[float]] = []
    for row_text in re.split(r";|\n", inner):
        row_text = row_text.strip()
        if not row_text:
            continue
        tokens = row_text.replace(",", " ").split()
        rows.append([float(_eval_scalar(token, values, context)) for token in tokens])

    width = max(len(row) for row in rows)
    matrix = np.full((len(rows), width), np.nan, dtype=float)
    for row_index, row in enumerate(rows):
        matrix[row_index, : len(row)] = row
    if matrix.shape[0] == 1:
        return matrix.ravel()
    return matrix


def _assign_indexed(current: Any, index: str, value: Any) -> np.ndarray:
    parts = [part.strip() for part in index.strip("()").split(",")]
    array = np.array(current, dtype=float, copy=True) if current is not None else np.empty((0, 0))
    value_array = np.atleast_1d(np.array(value, dtype=float))

    if len(parts) == 2 and parts[0] == ":":
        col = int(parts[1]) - 1
        rows = value_array.size
        new_array = np.full((max(array.shape[0], rows), max(array.shape[1] if array.ndim == 2 else 0, col + 1)), np.nan)
        if array.size:
            new_array[: array.shape[0], : array.shape[1]] = array
        new_array[:rows, col] = value_array
        return new_array

    if all(part.isdigit() for part in parts):
        zero_based = [int(part) - 1 for part in parts]
        if len(zero_based) == 1:
            new_array = np.full((max(array.size, zero_based[0] + 1),), np.nan)
            if array.size:
                new_array[: array.size] = array.ravel()
            new_array[zero_based[0]] = float(value_array.ravel()[0])
            return new_array
        row, col = zero_based
        new_array = np.full((max(array.shape[0], row + 1), max(array.shape[1] if array.ndim == 2 else 0, col + 1)), np.nan)
        if array.size:
            if array.ndim == 1:
                new_array[: array.size, 0] = array
            else:
                new_array[: array.shape[0], : array.shape[1]] = array
        new_array[row, col] = float(value_array.ravel()[0])
        return new_array

    raise ValueError(f"unsupported MATLAB index: {index}")


def _mirror_and_sort_lateral_stop_table(table: Any) -> np.ndarray:
    """Apply the MATLAB K_STy_Table mirror/sort post-processing."""

    base = np.asarray(table, dtype=float)
    mirrored = np.vstack([base, -base[1:, :]])
    return mirrored[np.argsort(mirrored[:, 0])]


def _eval_scalar(expression: str, values: dict[str, Any], context: dict[str, float]) -> float:
    expression = expression.strip()
    expression = expression.replace("^", "**")
    for key, value in context.items():
        expression = expression.replace(key, str(value))
    for name, value in values.items():
        if np.isscalar(value):
            expression = expression.replace(f"Par_Vehicle.{name}", str(value))

    node = ast.parse(expression, mode="eval")
    return float(_eval_node(node.body))


def _eval_node(node: ast.AST) -> float:
    binary_ops = {
        ast.Add: operator.add,
        ast.Sub: operator.sub,
        ast.Mult: operator.mul,
        ast.Div: operator.truediv,
        ast.Pow: operator.pow,
    }
    unary_ops = {
        ast.UAdd: operator.pos,
        ast.USub: operator.neg,
    }

    if isinstance(node, ast.Constant) and isinstance(node.value, (int, float)):
        return float(node.value)
    if isinstance(node, ast.BinOp) and type(node.op) in binary_ops:
        return float(binary_ops[type(node.op)](_eval_node(node.left), _eval_node(node.right)))
    if isinstance(node, ast.UnaryOp) and type(node.op) in unary_ops:
        return float(unary_ops[type(node.op)](_eval_node(node.operand)))
    raise ValueError(f"unsupported scalar expression: {ast.dump(node)}")
