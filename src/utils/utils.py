from collections.abc import Generator
import numpy as np
from typing import TypeVar

T = TypeVar("T")


def print_rounded_np(m):
    """Prints a numpy array rounded to 4 decimals"""
    print(np.round(m, 4))


def window_iter(array: list[T], window_size: int) -> Generator[list[T], None, None]:
    """
    Like rust, its an iterator that traverses the array in windows of size window_size
    """
    for idx in range(len(array) - window_size + 1):
        yield array[idx : idx + window_size]
