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
This script is needed for buildtime integrity routine.
It calculates and embeds HMAC and other needed stuff for in terms of FIPS 140-3
"""
import os
import sys
from IntegrityRoutine import IntegrityRoutine
from Utils import Utils
from log import log

__author__ = "Vadym Stupakov"
__copyright__ = "Copyright (c) 2017 Samsung Electronics"
__credits__ = ["Vadym Stupakov"]
__version__ = "1.0"
__maintainer__ = "Vadym Stupakov"
__email__ = "v.stupakov@samsung.com"
__status__ = "Production"

"""
The following lists contain object files as the module components within
crypto boundary according to FIPS 140-3 requirements. The components will
be verified in the frame of the integrity check.
The fingerprint value is embedded into fips140_out.o. To save integrity
the content of the object file will be skipped immediately at the fingerprint
calculation.
"""

fingerprint_obj_file = "fips140_out.o"

list_obj_files_skc = [
    "fips140_integrity.o",
    "fips140_post.o",
    "fips140_test.o",
    "fips140_3_services.o",
    "api.o",
    "cipher.o",
    "algapi.o",
    "scatterwalk.o",
    "skcipher.o",
    "ahash.o",
    "shash.o",
    "hmac.o",
    "sha1_generic.o",
    "sha256_generic.o",
    "sha512_generic.o",
    "ecb.o",
    "cbc.o",
    "aes_generic.o",
    "../lib/crypto/aes.o",
    "../lib/crypto/sha256.o",
    "../lib/crypto/sha1.o",
    fingerprint_obj_file
]

list_obj_files_skc_ce = [
    "aes-ce-core.o",
    "aes-ce-glue.o",
    "aes-ce.o",
    "aes-glue-ce.o",
    "sha256-core.o",
    "sha256-glue.o",
    "sha2-ce-core.o",
    "sha2-ce-glue.o",
    "sha1-ce-glue.o",
    "sha1-ce-core.o"
]

if __name__ == "__main__":

    if len(sys.argv) != 4:
        log.info(f"Usage {sys.argv[0]} [elf_file] [path to SKC *.o files]"
                 "[path to SKC-CE *.o files]")
        sys.exit(-1)

    elf_file = os.path.abspath(sys.argv[1])
    relative_path_to_skc_obj = sys.argv[2]
    relative_path_to_skc_ce_obj = sys.argv[3]

    utils = Utils()
    utils.paths_exists([elf_file])

    obj_files_full_path = []

    list_obj_files_skc.remove(fingerprint_obj_file)
    obj_files_full_path.extend([ os.path.join(relative_path_to_skc_obj, f_gen)
                                for f_gen in list_obj_files_skc ])
    obj_files_full_path.extend([ os.path.join(relative_path_to_skc_ce_obj, f_ce)
                                for f_ce in list_obj_files_skc_ce ])
    for f in obj_files_full_path:
        if not os.path.exists(f):
            raise ValueError(f"Object file {f} doesn`t exists")

    log.set_log_level(log.level_info)
    integrity = IntegrityRoutine(elf_file)

    sec_sym = integrity.get_canister_symbols(obj_files_full_path)
    integrity.make_integrity(sec_sym)

    sys.exit(0)
