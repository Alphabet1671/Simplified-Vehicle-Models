"""Frame writers: one ffmpeg pipe, a PNG sequence, and a null writer for tests.

A writer receives ``(height, width, 3)`` ``uint8`` RGB frames in order. The ffmpeg
writer starts one process for the whole video and writes raw frames to its stdin;
the file appears under its final name only when ffmpeg exits cleanly, so a failed
encode never leaves a plausible-looking video behind.
"""
from __future__ import annotations

import hashlib
import os
import shutil
import subprocess
import tempfile
from pathlib import Path
from typing import Protocol

import numpy as np

__all__ = ["EncoderUnavailable", "FrameWriter", "FfmpegFrameWriter", "PngSequenceWriter",
           "NullFrameWriter", "ffmpeg_command", "find_ffmpeg"]


class EncoderUnavailable(RuntimeError):
    """ffmpeg is missing or failed; the message carries the stderr tail."""


class FrameWriter(Protocol):
    def write(self, rgb: np.ndarray) -> None: ...

    def close(self) -> None: ...


def find_ffmpeg() -> str:
    path = shutil.which("ffmpeg")
    if path is None:
        raise EncoderUnavailable(
            "ffmpeg was not found on PATH; install ffmpeg or use --png-sequence DIR")
    return path


def _check_frame(rgb: np.ndarray, width: int, height: int) -> np.ndarray:
    frame = np.asarray(rgb)
    if frame.dtype != np.uint8 or frame.shape != (height, width, 3):
        raise ValueError(f"a frame must be uint8 with shape ({height}, {width}, 3); "
                         f"got {frame.dtype} {frame.shape}")
    return np.ascontiguousarray(frame)


def ffmpeg_command(executable: str, output, width: int, height: int, frame_rate: float,
                   crf: int = 18, preset: str = "medium") -> list[str]:
    """The exact ffmpeg argument list (raw RGB in, H.264 yuv420p out)."""

    return [executable, "-hide_banner", "-loglevel", "error", "-y",
            "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{width}x{height}",
            "-r", f"{frame_rate:g}", "-i", "-", "-an",
            "-c:v", "libx264", "-preset", str(preset), "-crf", str(int(crf)),
            "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(output)]


class FfmpegFrameWriter:
    """Encode frames to an H.264 file through one ffmpeg process."""

    def __init__(self, path, width: int, height: int, frame_rate: float = 120.0,
                 crf: int = 18, preset: str = "medium"):
        if width % 2 or height % 2:
            raise ValueError("yuv420p needs an even width and height")
        if not (np.isfinite(frame_rate) and frame_rate > 0.0):
            raise ValueError("frame_rate must be positive and finite")
        executable = find_ffmpeg()
        self.path = Path(path)
        self.width, self.height = int(width), int(height)
        self.frames = 0
        self._partial = self.path.with_name(self.path.stem + ".partial" + self.path.suffix)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._log = tempfile.TemporaryFile()
        command = ffmpeg_command(executable, self._partial, self.width, self.height,
                                 frame_rate, crf, preset)
        try:
            self._process = subprocess.Popen(command, stdin=subprocess.PIPE,
                                             stdout=subprocess.DEVNULL, stderr=self._log)
        except OSError as error:
            self._log.close()
            raise EncoderUnavailable(f"ffmpeg could not be started: {error}") from error
        self._closed = False

    def _stderr_tail(self, limit: int = 1500) -> str:
        self._log.flush()
        self._log.seek(0)
        return self._log.read().decode("utf-8", "replace")[-limit:].strip()

    def _finish_process(self) -> int:
        """Close the pipe, wait for ffmpeg, and return its exit status."""

        self._closed = True
        try:
            self._process.stdin.close()
        except OSError:
            pass
        return self._process.wait()

    def _failure(self, reason: str) -> EncoderUnavailable:
        """Build the error (with the stderr tail), then drop the partial file and the log."""

        tail = self._stderr_tail() or "no stderr output"
        if self._partial.exists():
            self._partial.unlink()
        self._log.close()
        return EncoderUnavailable(f"ffmpeg {reason}: {tail}")

    def write(self, rgb: np.ndarray) -> None:
        if self._closed:
            raise RuntimeError("the writer is closed")
        frame = _check_frame(rgb, self.width, self.height)
        try:
            self._process.stdin.write(frame.data)
        except OSError:
            status = self._finish_process()
            raise self._failure(f"exited with status {status}") from None
        self.frames += 1

    def close(self) -> None:
        if self._closed:
            return
        status = self._finish_process()
        if status != 0:
            raise self._failure(f"exited with status {status}")
        os.replace(self._partial, self.path)
        self._log.close()


class PngSequenceWriter:
    """Write ``frame_000000.png`` ... into a directory (the deterministic fallback)."""

    def __init__(self, directory, width: int, height: int):
        from PIL import Image  # noqa: F401 - fail early if Pillow is missing

        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True)
        self.width, self.height = int(width), int(height)
        self.frames = 0

    def write(self, rgb: np.ndarray) -> None:
        from PIL import Image

        frame = _check_frame(rgb, self.width, self.height)
        Image.fromarray(frame, "RGB").save(self.directory / f"frame_{self.frames:06d}.png")
        self.frames += 1

    def close(self) -> None:
        return None


class NullFrameWriter:
    """Count frames and hash their bytes; keeps nothing else."""

    def __init__(self, width: int | None = None, height: int | None = None):
        self.width, self.height = width, height
        self.frames = 0
        self.shapes: set[tuple] = set()
        self.dtypes: set[str] = set()
        self._hash = hashlib.sha256()
        self.closed = False

    def write(self, rgb: np.ndarray) -> None:
        frame = np.asarray(rgb)
        if self.width is not None:
            frame = _check_frame(frame, self.width, self.height)
        self.shapes.add(frame.shape)
        self.dtypes.add(str(frame.dtype))
        self._hash.update(np.ascontiguousarray(frame).tobytes())
        self.frames += 1

    def close(self) -> None:
        self.closed = True

    @property
    def digest(self) -> str:
        return "sha256:" + self._hash.hexdigest()
