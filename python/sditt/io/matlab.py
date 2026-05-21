from __future__ import annotations

import io
import struct
import zlib
from dataclasses import dataclass
from pathlib import Path
from typing import Any, BinaryIO

import numpy as np


MI_INT8 = 1
MI_UINT8 = 2
MI_INT16 = 3
MI_UINT16 = 4
MI_INT32 = 5
MI_UINT32 = 6
MI_SINGLE = 7
MI_DOUBLE = 9
MI_INT64 = 12
MI_UINT64 = 13
MI_MATRIX = 14
MI_COMPRESSED = 15
MI_UTF8 = 16
MI_UTF16 = 17
MI_UTF32 = 18

MX_CELL_CLASS = 1
MX_STRUCT_CLASS = 2
MX_CHAR_CLASS = 4
MX_SPARSE_CLASS = 5

DTYPES = {
    MI_INT8: ("i1", 1),
    MI_UINT8: ("u1", 1),
    MI_INT16: ("<i2", 2),
    MI_UINT16: ("<u2", 2),
    MI_INT32: ("<i4", 4),
    MI_UINT32: ("<u4", 4),
    MI_SINGLE: ("<f4", 4),
    MI_DOUBLE: ("<f8", 8),
    MI_INT64: ("<i8", 8),
    MI_UINT64: ("<u8", 8),
    MI_UTF8: ("u1", 1),
    MI_UTF16: ("<u2", 2),
    MI_UTF32: ("<u4", 4),
}


@dataclass(frozen=True)
class MatFile:
    """Loaded MATLAB v5 file."""

    path: Path
    header: str
    variables: dict[str, Any]


@dataclass(frozen=True)
class MatSummary:
    """Lightweight description of variables in a MATLAB file."""

    path: Path
    header: str
    variables: dict[str, dict[str, Any]]


@dataclass(frozen=True)
class SparseMatrixData:
    """Sparse matrix coordinates from a MATLAB v5 sparse array."""

    shape: tuple[int, ...]
    ir: np.ndarray
    jc: np.ndarray
    data: np.ndarray

    @property
    def nnz(self) -> int:
        return int(self.data.size)

    def to_dense(self) -> np.ndarray:
        dense = np.zeros(self.shape, dtype=self.data.dtype)
        for col in range(len(self.jc) - 1):
            start = int(self.jc[col])
            stop = int(self.jc[col + 1])
            rows = self.ir[start:stop]
            dense[rows, col] = self.data[start:stop]
        return dense


def load_mat_file(path: str | Path) -> MatFile:
    """Read a MATLAB v5 ``.mat`` file.

    The reader is deliberately small but covers the data forms used in the
    current SDITT artifacts: numeric arrays, structs, cells, chars, compressed
    elements, and MATLAB sparse matrices. It is a data loading layer only; no
    physical interpretation is performed here.
    """

    file_path = Path(path)
    with file_path.open("rb") as stream:
        header = stream.read(128)
        description = header[:116].decode("latin1", errors="replace").strip()
        variables = _read_elements(stream)
    return MatFile(path=file_path, header=description, variables=variables)


def summarize_mat_file(path: str | Path) -> MatSummary:
    file_path = Path(path)
    with file_path.open("rb") as stream:
        header = stream.read(128)
        description = header[:116].decode("latin1", errors="replace").strip()
        variables = _read_element_summaries(stream)
    return MatSummary(path=file_path, header=description, variables=variables)


def describe_value(value: Any) -> dict[str, Any]:
    if isinstance(value, np.ndarray):
        return {
            "type": "ndarray",
            "shape": tuple(int(part) for part in value.shape),
            "dtype": str(value.dtype),
        }
    if isinstance(value, SparseMatrixData):
        return {
            "type": "sparse",
            "shape": value.shape,
            "dtype": str(value.data.dtype),
            "nnz": value.nnz,
        }
    if isinstance(value, dict):
        return {
            "type": "struct",
            "fields": sorted(value.keys()),
        }
    if isinstance(value, list):
        return {
            "type": "cell",
            "length": len(value),
        }
    if isinstance(value, str):
        return {
            "type": "char",
            "length": len(value),
        }
    return {"type": type(value).__name__}


def _read_elements(stream: BinaryIO) -> dict[str, Any]:
    variables: dict[str, Any] = {}
    while True:
        tag = _read_tag(stream)
        if tag is None:
            break
        data_type, nbytes, small_data = tag
        payload = small_data if small_data is not None else stream.read(nbytes)
        if data_type != MI_COMPRESSED:
            _skip_padding(stream, nbytes)

        if data_type == MI_COMPRESSED:
            variables.update(_read_elements(io.BytesIO(zlib.decompress(payload))))
        elif data_type == MI_MATRIX:
            name, value = _parse_matrix(io.BytesIO(payload))
            variables[name] = value
    return variables


def _read_element_summaries(stream: BinaryIO) -> dict[str, dict[str, Any]]:
    variables: dict[str, dict[str, Any]] = {}
    while True:
        tag = _read_tag(stream)
        if tag is None:
            break
        data_type, nbytes, small_data = tag
        payload = small_data if small_data is not None else stream.read(nbytes)
        if data_type != MI_COMPRESSED:
            _skip_padding(stream, nbytes)

        if data_type == MI_COMPRESSED:
            name, summary = _read_compressed_matrix_summary(payload)
            variables[name] = summary
        elif data_type == MI_MATRIX:
            name, summary = _parse_matrix_summary(io.BytesIO(payload))
            variables[name] = summary
    return variables


def _read_compressed_matrix_summary(payload: bytes) -> tuple[str, dict[str, Any]]:
    # MATLAB commonly stores one compressed miMATRIX per variable. For summaries
    # we only need the front matter, so inflate a bounded prefix instead of
    # expanding multi-hundred-MB matrix payloads.
    for limit in (65_536, 262_144, 1_048_576, 8_388_608):
        decompressor = zlib.decompressobj()
        prefix = decompressor.decompress(payload, limit)
        try:
            tag = _read_tag(io.BytesIO(prefix))
            if tag is None:
                continue
            data_type, nbytes, small_data = tag
            if data_type != MI_MATRIX:
                continue
            matrix_payload = small_data if small_data is not None else prefix[8:]
            return _parse_matrix_summary(io.BytesIO(matrix_payload))
        except (IndexError, ValueError, struct.error):
            continue
    decompressed = zlib.decompress(payload)
    tag = _read_tag(io.BytesIO(decompressed))
    if tag is None:
        return "", {"type": "unknown", "shape": ()}
    data_type, _, small_data = tag
    if data_type != MI_MATRIX:
        return "", {"type": f"mi{data_type}", "shape": ()}
    matrix_payload = small_data if small_data is not None else decompressed[8:]
    return _parse_matrix_summary(io.BytesIO(matrix_payload))


def _parse_matrix(stream: BinaryIO) -> tuple[str, Any]:
    flags = _read_numeric_element(stream)
    class_id = int(flags[0]) & 0xFF if flags.size else 0
    dimensions = tuple(int(part) for part in _read_numeric_element(stream).ravel())
    name_element = _read_raw_element(stream)
    name = name_element.decode("utf-8", errors="replace")

    if class_id == MX_STRUCT_CLASS:
        return name, _parse_struct(stream, dimensions)
    if class_id == MX_CELL_CLASS:
        return name, _parse_cell(stream, dimensions)
    if class_id == MX_CHAR_CLASS:
        return name, _parse_char(stream)
    if class_id == MX_SPARSE_CLASS:
        return name, _parse_sparse(stream, dimensions)

    real = _read_numeric_element(stream)
    if _has_more(stream):
        imag = _read_numeric_element(stream)
        real = real.astype(complex) + 1j * imag
    return name, _reshape_matlab(real, dimensions)


def _parse_matrix_summary(stream: BinaryIO) -> tuple[str, dict[str, Any]]:
    flags = _read_numeric_element(stream)
    class_id = int(flags[0]) & 0xFF if flags.size else 0
    nzmax = int(flags[1]) if flags.size > 1 else 0
    dimensions = tuple(int(part) for part in _read_numeric_element(stream).ravel())
    name_element = _read_raw_element(stream)
    name = name_element.decode("utf-8", errors="replace")
    class_name = {
        MX_CELL_CLASS: "cell",
        MX_STRUCT_CLASS: "struct",
        MX_CHAR_CLASS: "char",
        MX_SPARSE_CLASS: "sparse",
    }.get(class_id, "numeric")

    summary: dict[str, Any] = {
        "type": class_name,
        "shape": dimensions,
        "class_id": class_id,
    }
    if class_id == MX_SPARSE_CLASS:
        summary["nzmax"] = nzmax
    if class_id not in {MX_CELL_CLASS, MX_STRUCT_CLASS, MX_CHAR_CLASS, MX_SPARSE_CLASS} and _has_more(stream):
        try:
            tag = _read_tag(stream)
            if tag is not None:
                dtype, nbytes, small_data = tag
                payload_size = len(small_data) if small_data is not None else nbytes
                summary["dtype_tag"] = dtype
                summary["payload_bytes"] = payload_size
        except ValueError:
            pass
    return name, summary


def _parse_struct(stream: BinaryIO, dimensions: tuple[int, ...]) -> dict[str, Any]:
    field_name_length = int(_read_numeric_element(stream).ravel()[0])
    field_names_raw = _read_raw_element(stream)
    field_names = [
        field_names_raw[index : index + field_name_length].split(b"\x00", 1)[0].decode(
            "utf-8", errors="replace"
        )
        for index in range(0, len(field_names_raw), field_name_length)
    ]
    count = int(np.prod(dimensions)) if dimensions else 1
    values_by_field = {field_name: [] for field_name in field_names}
    for _ in range(count):
        for field_name in field_names:
            tag = _read_tag(stream)
            if tag is None:
                values_by_field[field_name].append(None)
                continue
            data_type, nbytes, small_data = tag
            payload = small_data if small_data is not None else stream.read(nbytes)
            _skip_padding(stream, nbytes)
            if data_type == MI_MATRIX:
                _, value = _parse_matrix(io.BytesIO(payload))
            elif data_type == MI_COMPRESSED:
                nested = _read_elements(io.BytesIO(zlib.decompress(payload)))
                value = nested
            else:
                value = None
            values_by_field[field_name].append(value)

    if count == 1:
        return {field_name: values[0] for field_name, values in values_by_field.items()}
    return values_by_field


def _parse_cell(stream: BinaryIO, dimensions: tuple[int, ...]) -> list[Any]:
    count = int(np.prod(dimensions)) if dimensions else 1
    cells: list[Any] = []
    for _ in range(count):
        tag = _read_tag(stream)
        if tag is None:
            cells.append(None)
            continue
        data_type, nbytes, small_data = tag
        payload = small_data if small_data is not None else stream.read(nbytes)
        _skip_padding(stream, nbytes)
        if data_type == MI_MATRIX:
            _, value = _parse_matrix(io.BytesIO(payload))
            cells.append(value)
        else:
            cells.append(payload)
    return cells


def _parse_char(stream: BinaryIO) -> str:
    data_type, payload = _read_typed_payload(stream)
    if data_type == MI_UTF16:
        return payload.decode("utf-16le", errors="replace").rstrip("\x00")
    if data_type == MI_UTF32:
        return payload.decode("utf-32le", errors="replace").rstrip("\x00")
    return payload.decode("utf-8", errors="replace").rstrip("\x00")


def _parse_sparse(stream: BinaryIO, dimensions: tuple[int, ...]) -> SparseMatrixData:
    ir = _read_numeric_element(stream).astype(np.int64)
    jc = _read_numeric_element(stream).astype(np.int64)
    data = _read_numeric_element(stream)
    return SparseMatrixData(shape=dimensions, ir=ir, jc=jc, data=data)


def _read_numeric_element(stream: BinaryIO) -> np.ndarray:
    data_type, payload = _read_typed_payload(stream)
    if data_type not in DTYPES:
        return np.frombuffer(payload, dtype=np.uint8).copy()
    dtype, item_size = DTYPES[data_type]
    usable = len(payload) - (len(payload) % item_size)
    if usable <= 0:
        return np.empty((0,), dtype=np.dtype(dtype))
    return np.frombuffer(payload[:usable], dtype=np.dtype(dtype)).copy()


def _read_raw_element(stream: BinaryIO) -> bytes:
    _, payload = _read_typed_payload(stream)
    return payload


def _read_typed_payload(stream: BinaryIO) -> tuple[int, bytes]:
    tag = _read_tag(stream)
    if tag is None:
        return 0, b""
    data_type, nbytes, small_data = tag
    payload = small_data if small_data is not None else stream.read(nbytes)
    _skip_padding(stream, nbytes)
    return data_type, payload


def _read_tag(stream: BinaryIO) -> tuple[int, int, bytes | None] | None:
    raw = stream.read(8)
    if not raw:
        return None
    if len(raw) < 8:
        raise ValueError("truncated MATLAB data element tag")
    first, second = struct.unpack("<II", raw)
    small_nbytes = first >> 16
    small_type = first & 0xFFFF
    if small_nbytes:
        return small_type, small_nbytes, raw[4 : 4 + small_nbytes]
    return first, second, None


def _skip_padding(stream: BinaryIO, nbytes: int) -> None:
    padding = (8 - (nbytes % 8)) % 8
    if padding:
        stream.seek(padding, io.SEEK_CUR)


def _reshape_matlab(values: np.ndarray, dimensions: tuple[int, ...]) -> np.ndarray:
    if not dimensions:
        return values
    if int(np.prod(dimensions)) != values.size:
        return values
    return values.reshape(dimensions, order="F")


def _has_more(stream: BinaryIO) -> bool:
    position = stream.tell()
    stream.seek(0, io.SEEK_END)
    end = stream.tell()
    stream.seek(position)
    return position < end
