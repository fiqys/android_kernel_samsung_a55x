#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0
#
# In Samsung R&D Institute Ukraine, LLC (SRUKR) under a contract between
# Samsung R&D Institute Ukraine, LLC (Kyiv, Ukraine)
# and "Samsung Electronics Co", Ltd (Seoul, Republic of Korea)
# Copyright: (c) Samsung Electronics Co, Ltd 2024. All rights reserved.
#
# -*- coding: utf-8 -*-

"""
Module Utils contains Utils class with general purpose helper functions.
"""

import struct
import os
from itertools import chain

__author__ = "Vadym Stupakov"
__copyright__ = "Copyright (c) 2017 Samsung Electronics"
__credits__ = ["Vadym Stupakov"]
__version__ = "1.0"
__maintainer__ = "Vadym Stupakov"
__email__ = "v.stupakov@samsung.com"
__status__ = "Production"


class Utils:
    """
    Utils class with general purpose helper functions.
    """
    @staticmethod
    def flatten(alist):
        """
        Make list from sub lists.
        :param alist: any list: [[item1, item2], [item3, item4], ..., [itemN, itemN+1]].
        :return: [item1, item2, item3, item4, ..., itemN, itemN+1].
        """

        if not isinstance(alist, list):
            raise ValueError("Wrong input data type.")

        return list(chain.from_iterable(alist))

    @staticmethod
    def pairwise(iterable):
        """
        Iter over two elements: [s0, s1, s2, s3, ..., sN] -> (s0, s1), (s2, s3), ..., (sN, sN+1).
        :param iterable: iterable object with items to be paired.
        :return: (s0, s1), (s2, s3), ..., (sN, sN+1).
        """
        value = iter(iterable)
        return zip(value, value)

    @staticmethod
    def paths_exists(path_list):
        """
        Check if path exist, otherwise raise FileNotFoundError exception.
        :param path_list: list of paths.
        """
        for path in path_list:
            if not os.path.exists(path):
                raise FileNotFoundError("File: \"" + path + "\" doesn't exist!\n")

    @staticmethod
    def to_int(value, base=16):
        """
        Converts string to int.
        :param value: string or int.
        :param base: string base int.
        :return: integer value.
        """
        if isinstance(value, int):
            return value

        if isinstance(value, str):
            return int(value.strip(), base)

        raise ValueError("Wrong input data type.")

    def to_bytearray(self, value):
        """
        Converts list to bytearray with block size 8 byte.
        :param value: list of integers or bytearray or int.
        :return: bytes.
        """
        if isinstance(value, (bytearray, bytes)):
            return value

        if isinstance(value, list):
            value = self.flatten(value)
            return struct.pack(f"{len(value)}Q", *value)

        if isinstance(value, int):
            return struct.pack("Q", value)

        raise ValueError("Wrong input data type.")

    @staticmethod
    def human_size(nbytes):
        """
        Print in human readable.
        :param nbytes: number of bytes.
        :return: human readable string. For instance: 0x26a5d (154.6 K).
        """
        raw = nbytes
        suffixes = ("B", "K", "M")
        i = 0
        while nbytes >= 1024 and i < len(suffixes) - 1:
            nbytes /= 1024.
            i += 1
        fmt = f"{nbytes:.1f}".rstrip("0").rstrip(".")
        return f"{hex(raw)} ({fmt} {suffixes[i]})"

    def byte_int_to_hex_str(self, value, prefix=True):
        """
        Convert byte size integer value to hex string.
        :param value: integer value.
        :param prefix: prefix flag.
        :return: string.
        """
        str_hex = f"{self.to_int(value):02x}"
        return "0x" + str_hex if prefix else str_hex

    def dump_to_int(self, dump_mem):
        """
        Convert 8 byte bytearray to int.
        :param dump_mem: dump bytearray.
        :return: integer value.
        """
        str_dump = ""
        for l_count in range(0, 8):
            str_dump += self.byte_int_to_hex_str(dump_mem[7 - l_count], False)
        return self.to_int(str_dump)
