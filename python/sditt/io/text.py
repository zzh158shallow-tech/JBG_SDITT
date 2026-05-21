from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np


@dataclass(frozen=True)
class NumericTextData:
    """Numeric data loaded from a whitespace/comma separated text file."""

    path: Path
    data: np.ndarray
    skipped_lines: tuple[str, ...]


def load_numeric_text(
    path: str | Path,
    *,
    comments: tuple[str, ...] = ("!", "#", "%"),
    delimiter: str | None = None,
) -> NumericTextData:
    """Load numeric rows from a text-like file.

    Non-numeric header/comment lines are preserved in ``skipped_lines``. This
    intentionally avoids assuming a single SDITT text dialect; profile ``.txt``
    files, mileage files, and simple excitation text files all pass through the
    same narrow reader.
    """

    file_path = Path(path)
    rows: list[list[float]] = []
    skipped: list[str] = []

    for raw_line in file_path.read_text(encoding="utf-8", errors="ignore").splitlines():
        line = raw_line.strip()
        if not line or line.startswith(comments):
            if line:
                skipped.append(raw_line)
            continue

        if "!" in line:
            line = line.split("!", 1)[0].strip()
        if "%" in line:
            line = line.split("%", 1)[0].strip()
        if not line:
            skipped.append(raw_line)
            continue

        parts = line.split(delimiter) if delimiter else line.replace(",", " ").split()
        try:
            rows.append([float(part) for part in parts])
        except ValueError:
            numeric_prefix: list[float] = []
            for part in parts:
                try:
                    numeric_prefix.append(float(part))
                except ValueError:
                    break
            if numeric_prefix:
                rows.append(numeric_prefix)
                skipped.append(raw_line)
            else:
                skipped.append(raw_line)

    if not rows:
        data = np.empty((0, 0), dtype=float)
    else:
        width = max(len(row) for row in rows)
        data = np.full((len(rows), width), np.nan, dtype=float)
        for row_index, row in enumerate(rows):
            data[row_index, : len(row)] = row

    return NumericTextData(path=file_path, data=data, skipped_lines=tuple(skipped))
