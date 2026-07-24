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
Module IntegrityRoutine Contains IntegrityRoutine class which performs build-time
preparations for Integrity Check.
"""

import hmac
import hashlib
import binascii
from itertools import groupby
from struct import unpack
from typing import List
from copy import deepcopy
from ELF import ELF, DEFAULT_ARM_INST_WIDTH, Symbol
from log import log

__author__ = "Vadym Stupakov"
__copyright__ = "Copyright (c) 2017 Samsung Electronics"
__credits__ = ["Vadym Stupakov"]
__version__ = "1.0"
__maintainer__ = "Vadym Stupakov"
__email__ = "v.stupakov@samsung.com"
__status__ = "Production"

MAX_NUMBER_OF_RECORDS_FOR_GAPS_IN_KERNEL = 4096

class IntegrityRoutine(ELF):
    """
    Utils for fips-integrity process.
    """
    def __init__(self, elf_file):
        ELF.__init__(self, elf_file)

    @staticmethod
    def __uniq_values(lst):
        """
        Return unique values.
        For instance: transforms [1, 2, 4, 3, 1] -> [2, 3, 4].
        :param lst: input list.
        :return: sorted lst with uniq values.
        """
        if len(lst) < 2:
            return lst
        lst.sort()
        return [k for k, v in groupby(lst) if len(list(v)) < 2]

    def get_reloc_gaps(self, relocs_list, start_addr, end_addr):
        """
        Return relocation gaps are in range of addresses.
        :param start_addr: start address :int.
        :param end_addr: end address: int.
        :return: list of exclude addr like [exclude_addr1, exclude_addr2, ...].
        """
        relocs_gaps = []
        all_relocs = self.get_relocs_for_symbol(relocs_list, start_addr, end_addr)
        for addr in all_relocs:
            relocs_gaps.extend(range(addr, addr + 8))
        return relocs_gaps

    def get_altinstruction_gaps(self, start_addr, end_addr, alt_instr_text):
        """
        Return altinstruction gaps are in range of addresses.
        :param start_addr: start address :int.
        :param end_addr: end address: int.
        :return: list of exclude addr like [exclude_alt_addr1, exclude_alt_addr2, ...].
        """
        return self.get_altinstructions(alt_instr_text, start_addr, end_addr)

    def get_jump_table_gaps(self, start_addr: int, end_addr: int, jump_table: list) -> list:
        """
        Return JT related gaps are in range of our module
        :param start_addr: start address.
        :param end_addr: end address.
        :param jump_table: full list (over whole kernel) of JT items.
        :return: list of addrs to be excluded [exclude_addr1, exclude_addr2, ...].
        """
        return self.get_jump_table_module(start_addr, end_addr, jump_table)

    def get_gaps(self, exclude_addrs):
        """
        Calculate gaps based on excluded addresses.
        :param exclude_addrs: list of addresses excluded from fingerprint calculation.
        :return: list of gaps.
        """
        gaps = []
        for addr in exclude_addrs:
            gaps.append(addr)
            gaps.append(addr+1)
        gaps_removed_equal = self.__uniq_values(gaps)
        return [[addr1, addr2] for addr1, addr2 in self.utils.pairwise(gaps_removed_equal)]

    def get_addrs_for_hmac(self, sec_sym_sequence, exclude_addrs):
        """
        Generate addresses for calculating HMAC.
        :param sec_sym_sequence: [[text_symbol1, ..., text_symbolN]],
                                  [rodata_symbol1, ..., rodata_symbolN]].
        :param exclude_addrs: [exclude_addr1, exclude_addr2, ..., exclude_addr3].
        :return: addresses for calculating HMAC: [[addr_start, addr_end],
                                                  [addr_start, addr_end], ...].
        """
        symbol_scope = []
        hmac_scope = []
        for symbol in sec_sym_sequence:
            for addr_one in range(symbol.addr, symbol.addr + symbol.size):
                symbol_scope.append(addr_one)

        symbol_scope.sort()
        symbol_scope_final = [el for el, _ in groupby(symbol_scope)]

        # Exclude addresses from HMAC
        i_exclude = 0
        for sym_addr in symbol_scope_final:
            while i_exclude < len(exclude_addrs):
                if sym_addr < exclude_addrs[i_exclude]:
                    hmac_scope.append(sym_addr)
                    hmac_scope.append(sym_addr + 1)
                    break
                if sym_addr == exclude_addrs[i_exclude]:
                    break
                i_exclude += 1
            if i_exclude >= len(exclude_addrs):
                hmac_scope.append(sym_addr)
                hmac_scope.append(sym_addr + 1)
        hmac_removed_equal = self.__uniq_values(hmac_scope)
        return [[item1, item2] for item1, item2 in
                self.utils.pairwise(hmac_removed_equal) if item1 != item2]

    def embed_bytes(self, vaddr, in_bytes):
        """
        Write bytes to ELF file.
        :param vaddr: virtual address in ELF.
        :param in_bytes: byte array to write.
        """
        offset = self.vaddr_to_file_offset(vaddr)
        with open(self.get_elf_file(), "rb+") as elf_file:
            elf_file.seek(offset)
            elf_file.write(in_bytes)

    def __update_hmac(self, hmac_obj, file_obj, file_offset_start, file_offset_end):
        """
        Update hmac from addrstart tp addr_end.
        FIXMI: it needs to implement this function via fixed block size.
        :param file_offset_start: could be string or int.
        :param file_offset_end:   could be string or int.
        """
        file_offset_start = self.utils.to_int(file_offset_start)
        file_offset_end = self.utils.to_int(file_offset_end)
        file_obj.seek(self.vaddr_to_file_offset(file_offset_start))
        block_size = file_offset_end - file_offset_start
        msg = file_obj.read(block_size)
        hmac_obj.update(msg)

    def get_hmac(self, offset_sequence, key, output_type="byte"):
        """
        Calculate HMAC.
        :param offset_sequence: start and end addresses sequence
                                [addr_start, addr_end], [addr_start, addr_end], ...].
        :param key HMAC key: string value.
        :param output_type: string value. Could be "hex" or "byte".
        :return: bytearray or hex string.
        """
        digest = hmac.new(bytearray(key.encode("utf-8")), digestmod=hashlib.sha256)
        with open(self.get_elf_file(), "rb") as file:
            for addr_start, addr_end in offset_sequence:
                self.__update_hmac(digest, file, addr_start, addr_end)
        if output_type == "byte":
            return digest.digest()
        if output_type == "hex":
            return digest.hexdigest()

        raise ValueError("Wrong output type requested.")

    def get_canister_symbols(self, object_files: List[str]) -> List[Symbol]:
        """
        Derive list of symbols which are present in both kernel ELF and SKC object files.
        The list includes symbols from text and rodata related sections.
        :param object_files: list of of paths to object files.
        :param debug: flag for debug output.
        :return: Lists of symbols are present in kernel to be covered by Integrity Check.
        """

        substr_symnames_forbidden = []
        substr_sections_forbidden = [".init", ".data", ".exit", ".head", "Absolute",
                                     "Undefined", ".rodata.text", "__param", ".bss",
                                     "__modver", ".notes", "alloc_tags", ".discard",
                                     ".modinfo", ".rodata.__llvm_fs_discriminator__",
                                     ".BTF"]

        needed_sections_substr = ["rodata", "text"]

        symbols_obj = []
        for path in object_files:
            symbols_obj.extend(self.get_symbols_from_elf(path,
                                                         substr_symnames_forbidden,
                                                         substr_sections_forbidden,
                                                         needed_sections_substr))

        symbols_elf = deepcopy(self.get_kernel_symbols())

        canister_symbols = []
        for sym_obj in symbols_obj:
            elf_syms_to_be_removed = []
            for sym_elf in symbols_elf:
                if sym_elf == sym_obj:
                    canister_symbols.append(sym_elf)
                    elf_syms_to_be_removed.append(sym_elf)

            for item in elf_syms_to_be_removed:
                symbols_elf.remove(item)

        log.debug_low_details("Number of canister symbols in sections of SKC object files : "
                              f"{format(len(symbols_obj))}.")
        log.debug_low_details("Number of symbols in kernel ELF to be covered by IC        : "
                              f"{format(len(canister_symbols))}.")

        return canister_symbols

    def unite_borders(self, fields_scope):
        """
        Merge areas to be covered by fingerprint calculation.
        :param fields_scope: list of areas to be covered by fingerprint calculation.
        :return: list of merged areas.
        """
        if len(fields_scope) < 2:
            return fields_scope
        united_list = []
        united_list.extend(fields_scope[0])
        for i in range(1, len(fields_scope)):
            united_list.extend(fields_scope[i])
            if united_list[-2] == united_list[-3]:
                united_list.pop(-2)
                united_list.pop(-2)

        return [[item1, item2] for item1, item2 in
                self.utils.pairwise(united_list) if item1 != item2]

    def print_areas_info(self, sec_sym: List[Symbol],
                         addrs_for_hmac: List[List],
                         addrs_gaps: List[List]):
        """
        Print info about areas covered by IC and gaps.
        :param sec_sym: list of symbols.
        :param addrs_for_hmac: addresses are used for fingerprint calculation.
        :param addrs_gaps: list of gaps addresses [[start1, end1], .. [start2, end2]].
        """
        gaps_cover = 0
        log.debug_high_details("\nGaps :\n")
        for i, gap in enumerate(addrs_gaps):
            gaps_cover += (gap[1] - gap[0])
            log.debug_high_details(f"{i:4}| [{hex(gap[0])}, {hex(gap[1])}]")

        log.debug_high_details("\nSymbols to be covered by Integrity Check :\n")
        for i, symbol in enumerate(sec_sym):
            log.debug_high_details(f"{i + 1:<4}| {symbol.name:<72} {hex(symbol.addr):<25}"
                                   f"{symbol.type:<10} {symbol.bind:<12}"
                                   f" size: {hex(symbol.size):<10}")

        log.debug_high_details("\nAddress ranges were covered by integrity check:\n")
        area_covered_bytes = 0
        for i, addr_rec in enumerate(addrs_for_hmac):
            area_covered_bytes += (addr_rec[1] - addr_rec[0])
            log.debug_high_details(f"{i + 1:4}| [{hex(addr_rec[0])}, {hex(addr_rec[1])}]")

        percent_cover = ((100*area_covered_bytes) / (area_covered_bytes + gaps_cover))
        log.debug_high_details("\nTotal size of the Module, bytes           : "
                               f"{self.utils.human_size(area_covered_bytes + gaps_cover)}.")
        log.debug_high_details("Size of area covered by fingerprint, bytes : "
                               f"{self.utils.human_size(area_covered_bytes)}.")
        log.debug_high_details("Size of skipped area (as gaps), bytes      : "
                               f"{self.utils.human_size(gaps_cover)}.")
        log.debug_high_details("Cover ratio, %                             : "
                               f"{percent_cover:.4}%.")

    def dump_covered_bytes(self, vaddr_seq, out_file_bin, out_file_txt):
        """
        Dumps covered bytes.
        :param vaddr_seq: [[start1, end1], [start2, end2]] start - end sequence of covered bytes.
        :param out_file_bin: file where will be stored binary dumped bytes.
        :param out_file_txt: file where will be stored string dumped bytes.
        """
        with open(self.get_elf_file(), "rb") as elf_fp:
            with open(out_file_bin, "wb") as out_fp:
                with open(out_file_txt, mode="w", encoding="utf-8") as out_ft:
                    i = 0
                    for vaddr_start, vaddr_end, in vaddr_seq:
                        elf_fp.seek(self.vaddr_to_file_offset(vaddr_start))
                        block_size = vaddr_end - vaddr_start
                        dump_mem = elf_fp.read(block_size)
                        out_fp.write(dump_mem)
                        out_ft.write(f"\nArea cover {i} [{hex(vaddr_start)}, "
                                     f"{hex(vaddr_end)}], size = {hex(block_size)}:\n")
                        str_dump = ""
                        for l_count in range(0, block_size):
                            str_dump = (str_dump
                            + self.utils.byte_int_to_hex_str(dump_mem[l_count], False)
                            + " ")

                            if (l_count + 1) % 16 == 0:
                                str_dump = str_dump + "\n"
                        str_dump = str_dump + "\n"
                        out_ft.write(str_dump)
                        i += 1

    def get_relocations_for_ftrace(self, addr_start, addr_end):
        """
        Getting relocation table from output ELF file.
        :param addr_start: range start address.
        :param addr_end: range end address.
        :return: list of exclude addrs like [exclude_alt_addr1, exclude_alt_addr2, ...].
        """
        ftrace_tbl = []
        rela_sect_obj = self.get_section_by_name(".rela.dyn")
        if rela_sect_obj is None:
            return ftrace_tbl
        with open(self.get_elf_file(), "rb") as elf_fp:
            elf_fp.seek(self.vaddr_to_file_offset(rela_sect_obj.addr))
            i = 0
            while i < rela_sect_obj.size:
                dump_mem = elf_fp.read(8)
                r_offset = self.utils.dump_to_int(dump_mem)
                dump_mem = elf_fp.read(8)
                r_info = self.utils.dump_to_int(dump_mem) # pylint: disable=unused-variable
                dump_mem = elf_fp.read(8)
                r_addend = self.utils.dump_to_int(dump_mem)
                if addr_start <= r_offset < addr_end:
                    ftrace_tbl.append(r_addend)
                i += 24
        ftrace_tbl.sort()
        return ftrace_tbl

    def get_exclude_ftrace_addr(self, sec_sym, ftrace_tbl):
        """
        Getting excluded addresses from ftrace table for target symbols.
        :param sec_sym: list of symbols where the gaps will be searched.
        :param ftrace_tbl: list of ftrace items.
        :return: list of exclude addrs like [exclude_alt_addr1, exclude_alt_addr2, ...].
        """
        ftrace_addr_change = []
        if len(ftrace_tbl) == 0:
            return ftrace_addr_change
        i_ftrace = 0
        for symbol in sec_sym:
            addr_start = symbol.addr
            addr_end = symbol.addr + symbol.size
            while i_ftrace < len(ftrace_tbl):
                if ftrace_tbl[i_ftrace] >= addr_start and ftrace_tbl[i_ftrace] < addr_end:
                    for skip_addr in range(ftrace_tbl[i_ftrace], ftrace_tbl[i_ftrace] + 4):
                        ftrace_addr_change.append(skip_addr)
                elif ftrace_tbl[i_ftrace] >= addr_end:
                    break
                i_ftrace += 1
        return ftrace_addr_change

    def get_ftrace_gaps(self, sec_sym):
        """
        Determine list of gaps caused by ftrace instructions.
        :param sec_sym: list of symbols.
        :return: list of gaps.
        """
        ftrace_tbl = []
        start_mcount_loc = self.get_symbol_by_name("__start_mcount_loc")
        stop_mcount_loc = self.get_symbol_by_name("__stop_mcount_loc")
        if start_mcount_loc is not None and stop_mcount_loc is not None:
            ftrace_tbl = self.get_relocations_for_ftrace(start_mcount_loc.addr,
                                                         stop_mcount_loc.addr)
        return self.get_exclude_ftrace_addr(sec_sym, ftrace_tbl)

    def get_pac_scs_gaps(self, start_addr: int, size: int) -> List[int]:
        """
        Obtain addresses which can be modified in the frame of PAC/SCS instructions replacement.
        :param start_addr: start addr (ELF addr space) of the checked address range.
        :param size: size of the checked address range.
        :return: list of addrs to be excluded [exclude_addr1, exclude_addr2, ...]
        """
        pac_scs_instr_addrs = []
        pac_scs_opcodes = [
            0xd503233f, # PACIASP
            0xd50323bf, # AUTIASP
            0xf800865e, # SCS_PUSH
            0xf85f8e5e, # SCS_POP
        ]

        area_dump = self.get_data_by_vaddr(start_addr, size)
        if len(area_dump) % DEFAULT_ARM_INST_WIDTH != 0:
            raise ValueError(f"Checked size should be a multiple of {DEFAULT_ARM_INST_WIDTH}.")

        for opcode_idx in range(0, len(area_dump), DEFAULT_ARM_INST_WIDTH):
            opcode = unpack('<I', area_dump[opcode_idx : opcode_idx + DEFAULT_ARM_INST_WIDTH])
            if opcode[0] in pac_scs_opcodes:
                addrs_range = range(start_addr + opcode_idx,
                                    start_addr + opcode_idx + DEFAULT_ARM_INST_WIDTH)
                pac_scs_instr_addrs.extend(list(addrs_range))

        return pac_scs_instr_addrs

    def make_integrity(self, sec_sym: List[Symbol]):
        """
        Calculate HMAC and embed needed info.
        :param sec_sym: List of symbols to be covered by fingerprint.
        """

        relocs_text, relocs_rodata = self.get_relocs_text_rodata()
        alt_instr_text, alt_instr_rodata = self.get_text_rodata_altinstructions_lists()
        jump_table = self.get_jump_table_list()
        ftrace_exclude_addrs = self.get_ftrace_gaps(sec_sym)

        log.debug_low_details("Number of relocations (text/rodata) in the kernel ELF         : "
                              f"{len(relocs_text)}/{len(relocs_rodata)}.")
        log.debug_low_details("Number of instr. alternations (text/rodata) in the kernel ELF : "
                              f"{len(alt_instr_text)}/{len(alt_instr_rodata)}.")
        log.debug_low_details("Number of jumptable entries in the kernel ELF                 : "
                              f"{len(jump_table)}.")
        log.debug_low_details("Number of ftrace entries in the kernel ELF                    : "
                              f"{len(ftrace_exclude_addrs)}.")

        if len(alt_instr_rodata) != 0:
            log.info("\nAttention: .rodata relating section contains "
                     f"{len(alt_instr_rodata)} alt. instructions.\n")

        exclude_addrs = []

        if len(ftrace_exclude_addrs) != 0:
            exclude_addrs.extend(ftrace_exclude_addrs)

        if len(relocs_rodata) != 0:
            for symbol_rodata in self.get_symbols_by_section_name(sec_sym, "rodata"):
                exclude_addrs.extend(self.get_reloc_gaps(relocs_rodata, symbol_rodata.addr,
                                                         symbol_rodata.addr + symbol_rodata.size))

        if len(relocs_text) != 0:
            for symbol_text in self.get_symbols_by_section_name(sec_sym,"text"):
                exclude_addrs.extend(self.get_reloc_gaps(relocs_text, symbol_text.addr,
                                                         symbol_text.addr + symbol_text.size))

        if len(alt_instr_text) != 0:
            for symbol_text in self.get_symbols_by_section_name(sec_sym,"text"):
                exclude_addrs.extend(
                    self.get_altinstruction_gaps(symbol_text.addr,
                                                 symbol_text.addr + symbol_text.size,
                                                 alt_instr_text))

        if len(jump_table) != 0:
            for symbol_text in self.get_symbols_by_section_name(sec_sym, "text"):
                exclude_addrs.extend(self.get_jump_table_gaps(symbol_text.addr,
                                                              symbol_text.addr + symbol_text.size,
                                                              jump_table))

        for symbol_text in self.get_symbols_by_section_name(sec_sym, "text"):
            exclude_addrs.extend(self.get_pac_scs_gaps(symbol_text.addr, symbol_text.size))

        exclude_addrs.sort()
        exclude_addrs_no_matches = [ex for ex, _ in groupby(exclude_addrs)]

        hmac_fields = self.get_addrs_for_hmac(sec_sym, exclude_addrs_no_matches)
        addrs_for_hmac = self.unite_borders(hmac_fields)

        if len(addrs_for_hmac) >= MAX_NUMBER_OF_RECORDS_FOR_GAPS_IN_KERNEL:
            raise ValueError(f"ERROR: number of gaps records {len(addrs_for_hmac)}"
                             "exceeds allocated memory in kernel.")

        digest = self.get_hmac(addrs_for_hmac, "The quick brown fox jumps over the lazy dog")

        self.embed_bytes(self.get_symbol_by_name("buildtime_crypto_hmac").addr,
                         self.utils.to_bytearray(digest))

        self.embed_bytes(self.get_symbol_by_name("integrity_crypto_addrs").addr,
                         self.utils.to_bytearray(addrs_for_hmac))

        self.embed_bytes(self.get_symbol_by_name("crypto_buildtime_address").addr,
                         self.utils.to_bytearray(self.get_symbol_by_name(
                             "crypto_buildtime_address").addr))

        if log.level_is_enabled(log.level_debug_high_details):
            self.print_areas_info(sec_sym, addrs_for_hmac,
                                  self.get_gaps(exclude_addrs_no_matches))
            self.dump_covered_bytes(addrs_for_hmac, "covered_dump_for_crypto.bin",
                                    "covered_dump_for_crypto.txt")

        log.debug_low_details(f"The module HMAC : {binascii.hexlify(digest)}")
