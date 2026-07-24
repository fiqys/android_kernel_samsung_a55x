#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0
#
# In Samsung R&D Institute Ukraine, LLC (SRUKR) under a contract between
# Samsung R&D Institute Ukraine, LLC (Kyiv, Ukraine)
# and "Samsung Electronics Co", Ltd (Seoul, Republic of Korea)
# Copyright: (c) Samsung Electronics Co, Ltd 2025. All rights reserved.
#
# -*- coding: utf-8 -*-

"""
Logging implementation for Integrity Check routines.
"""

from typing import List, Dict

class log:
    """
    Logging functionality.
    """
    level_info = 0
    level_debug_low_details = 1
    level_debug_high_details = 2

    __log_level_max = level_debug_high_details = 2
    __log_level_min = level_info

    __current_log_level = level_info

    @staticmethod
    def set_log_level(log_level: int) -> None:
        """
        Set necessary logging level.
        :param log_level: can be one of "level_info", "level_debug_low_details",
                          "level_debug_high_details"
        :raises ValueError: if wrong level requested.
        """
        if log.__log_level_min > log_level > log.__log_level_max:
            raise ValueError("The log level is not applicable.")

        log.__current_log_level = log_level

    @staticmethod
    def level_is_enabled(req_log_level: int) -> bool:
        """
        The function to return True if the log output is allowed
        for the currently set of log level.
        :param req_log_level: log level of input message.
        :raises ValueError: if wrong level requested.
        :returns: True if the current log level is equal or higher then requested.
        """
        if log.__log_level_min > req_log_level > log.__log_level_max:
            raise ValueError("The log level is not applicable.")

        return log.__current_log_level >= req_log_level

    @staticmethod
    def common(req_log_level: int, *args: List, **kwargs: Dict) -> None:
        """
        Common logging function.
        :param req_log_level: log level of input message.
        """
        if log.level_is_enabled(req_log_level):
            print(*args, **kwargs)

    @staticmethod
    def info(*args: List, **kwargs: Dict) -> None:
        """ Info logging function. """
        log.common(log.level_info, *args, **kwargs)

    @staticmethod
    def debug_low_details(*args: List, **kwargs: Dict) -> None:
        """ Log debug low detail info. """
        log.common(log.level_debug_low_details, *args, **kwargs)

    @staticmethod
    def debug_high_details(*args: List, **kwargs: Dict) -> None:
        """ Log debug high detail info. """
        log.common(log.level_debug_high_details, *args, **kwargs)
