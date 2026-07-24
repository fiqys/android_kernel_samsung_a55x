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
Module ELF contains ELF, Symbol, Section classes for manipulation over ELF files.
It can parse, and change ELF file.
"""

import subprocess
import re
import os
import struct
import json
from collections import OrderedDict
from binascii import unhexlify
from typing import List, Literal
from math import ceil
from dataclasses import dataclass
from Utils import Utils
from log import log

__author__ = "Vadym Stupakov"
__copyright__ = "Copyright (c) 2017 Samsung Electronics"
__credits__ = ["Vadym Stupakov"]
__version__ = "1.0"
__maintainer__ = "Vadym Stupakov"
__email__ = "v.stupakov@samsung.com"
__status__ = "Production"

DEFAULT_NAME_JUMP_TABLE_START_SYM = "__start___jump_table"
DEFAULT_NAME_JUMP_TABLE_END_SYM = "__stop___jump_table"
DEFAULT_ARM_INST_WIDTH = 4


@dataclass
class SecJumpTableData:
    """
    Class to hold data of jump table record.
    """
    target_offset: int  = -1
    code: int = -1
    key: int = -1

    def __str__(self):
        return (f" target offset: {hex(self.target_offset):<10} code: {hex(self.code):<10}"
                f" key: {hex(self.key):<10}")

class Symbol:
    """
    Class of ELF symbol.
    """
    def __init__(self):
        self.utils = Utils()
        self.name = ""
        self.type = ""
        self.bind = ""
        self.section_name = ""
        self.addr = 0
        self.size = 0

    def __str__(self):
        return (f" {self.name:<40} {self.type:<10} {self.bind:<8} {self.section_name:<10}"
                f" {hex(self.addr):<12} {hex(self.size):<8}")

    def __eq__(self, other):
        if not isinstance(other, Symbol):
            raise ValueError("Comparison of wrong types.")

        return self.name == other.name and self.size == other.size


@dataclass
class Section:
    """
    Dataclass of ELF section.
    """
    utils: Utils = Utils()
    name: str = ""
    type: str = ""
    addr: int = 0
    offset: int = 0
    size: int = 0


class ELF:
    """
    Utils for manipulating over ELF
    """
    def __init__(self, elf_file):
        self.__elf_file = elf_file
        self.utils = Utils()
        self.__sections = OrderedDict()
        self.__symbols = OrderedDict()
        self.__symbols_kernel_elf = []
        self.__re_hexadecimal = r"\s*[0-9A-Fa-f]+\s*"
        self.__re_sec_name = r"\s*[._a-zA-Z]+\s*"
        self.__re_type = r"\s*[A-Z]+\s*"
        self.__altinstr_text = None
        self.__altinstr_rodata = None

        """
        To derive info from the kernel`s ELF and object files, it is necessary to use nm
        and readelf. The both should be from cross-toolchain which is used at the kernel
        build. So, let's take a correct tools from READELF environment variables,
        the variables are set by Makefile from the kernel source root directory.
        """
        self.__readelf_tool = os.environ["READELF"]

        self.jumptable_struct_format = "<iiQ"
        self.__jt_rec = []

        substr_symnames_forbidden_kernel = [ "$", ".cfi_jt", "ksymtab", "kstrtab", "__crc" ]
        substr_sections_forbidden_kernel = [".init", ".data", ".exit", ".head", "Absolute",
                                            "Undefined", ".rodata.text", "__param", ".bss",
                                            "__modver", ".notes", "BTF", "uh_ro", ".debug",
                                            ".eh", "kcrctab" ]
        needed_sections_substr = ["rodata", "text"]

        self.__symbols_kernel_elf = self.get_symbols_from_elf(self.__elf_file,
                                                substr_symnames_forbidden_kernel,
                                                substr_sections_forbidden_kernel,
                                                needed_sections_substr)
        if not self.__symbols_kernel_elf:
            raise ValueError("It seems input kernel ELF is stripped.")

    def get_kernel_symbols(self) -> List[Symbol]:
        """
        Return list of kernel`s symbols.
        """
        return self.__symbols_kernel_elf

    def get_raw_by_tool(self, tool_name, options):
        """
        Execute tool_name with options and print raw output.
        :param tool_name: path to tool.
        :param options: options of applied tool: ["opt1", "opt2", "opt3", ..., "optN"].
        :return: raw output as sring.
        """

        with subprocess.Popen(args=[tool_name] + options,
                              stdout=subprocess.PIPE,
                              stderr=subprocess.PIPE) as process:
            stdout, stderr = process.communicate()
            err_msg = stderr.decode("utf-8").strip()
            if process.returncode != 0 and ("error" in err_msg or "Error" in err_msg):
                raise ChildProcessError(stderr.decode("utf-8"))

        return stdout.decode("utf-8")

    def get_elf_file(self):
        """
        Return abspath of ELF file.
        :return: path to ELF file as string.
        """
        return os.path.abspath(self.__elf_file)

    def get_sections(self):
        """
        Get sections from ELF.
        :return: sections as dictionary {sec_addr : Section()}.
        """
        if len(self.__sections) == 0:
            sec_header = self.get_raw_by_tool(self.__readelf_tool,
                                              ["-SW", "--elf-output-style=GNU",
                                               self.__elf_file]).strip()
            secs = re.compile(r"^.*\[.*\](" + self.__re_sec_name + self.__re_type +
                              self.__re_hexadecimal +
                              self.__re_hexadecimal +
                              self.__re_hexadecimal + ")", re.MULTILINE)
            found = secs.findall(sec_header)
            for line in found:
                line = line.split()
                if len(line) == 5:
                    section = Section()
                    section.name = line[0]
                    section.type = line[1]
                    section.addr = int(line[2], 16)
                    section.offset = int(line[3], 16)
                    section.size = int(line[4], 16)
                    self.__sections[int(line[2], 16)] = section
            self.__sections = OrderedDict(sorted(self.__sections.items()))
        return self.__sections

    def get_rodata_text_scope(self):
        """
        Return text and rodata related sections.
        :return: lists ([text sections], [rodata sections]]).
        """
        raw_sections = self.get_raw_by_tool(self.__readelf_tool, ["-SW",
                                                                  "--elf-output-style=GNU",
                                                                  self.__elf_file]).strip()
        section_rodata = []
        section_text = []

        for line in raw_sections.splitlines():
            line_list = list(line.split())
            i = 0
            len_list = len(line_list)
            while i < len_list:
                if "." not in line_list[i]:
                    del line_list[i]
                    len_list = len(line_list)
                else:
                    break
            if len(line_list) >= 6:
                if line_list[0].strip().startswith(".rodata"):
                    if int(line_list[4].strip(), 16) != 0:
                        section_rodata.append([line_list[2].strip(), line_list[4].strip()])
                elif line_list[0].strip().startswith(".text"):
                    section_text.append([line_list[2].strip(), line_list[4].strip()])
        return section_text, section_rodata

    def get_symbols_from_elf(self,
                             path_to_file: str,
                             substr_symname_forbidden: List[str],
                             substr_secname_forbidden: List[str],
                             substr_secname_acceptable: List[str],
                             ) -> List[Symbol]:
        """
        Collect and filter data about symbols from the object file.
        :param path_to_file: path to object file.
        :param substr_symname_forbidden: substrings are forbidden in symbol names.
        :param substr_secname_forbidden: substrings are forbidden in section names.
        :param substr_secname_acceptable: substrings of target section names.
        :return: list of Symbol objects.
        """

        str_contains_substr = lambda str_val, substr_lst : any(substr in str_val
                                                              for substr in substr_lst)

        out_list = []
        non_categorized_syms = []

        symbols_json_str = self.get_raw_by_tool(self.__readelf_tool,
                                                ["-s", "--elf-output-style=JSON", path_to_file])
        symbols = json.loads(symbols_json_str)

        for symbol_rec in symbols[0]["Symbols"]:
            if (not str_contains_substr(symbol_rec["Symbol"]["Name"]["Value"],
                                        substr_symname_forbidden)
                and not str_contains_substr(symbol_rec["Symbol"]["Section"]["Value"],
                                            substr_secname_forbidden)):

                symbol = Symbol()
                symbol.addr = symbol_rec["Symbol"]["Value"]
                symbol.size = symbol_rec["Symbol"]["Size"]
                symbol.type = symbol_rec["Symbol"]["Type"]["Value"]
                symbol.bind = symbol_rec["Symbol"]["Binding"]["Value"]
                symbol.section_name = symbol_rec["Symbol"]["Section"]["Value"]
                symbol.name = symbol_rec["Symbol"]["Name"]["Value"]

                if str_contains_substr(symbol_rec["Symbol"]["Section"]["Value"],
                                       substr_secname_acceptable):
                    out_list.append(symbol)
                else:
                    non_categorized_syms.append(symbol)

        if non_categorized_syms:
            for symbol in non_categorized_syms:
                log.debug_high_details(symbol)

            log.debug_high_details("\nAttention: Non-categorized symbol is in the ELF file"
                f" {os.path.basename(path_to_file)}.")
            log.debug_high_details("It is necessary to clarify what the category it"
                " belongs to (text, rodata) and revise forbidden/acceptable list of the"
                " function input.")

        return out_list

    def filtered_addr_by_section(self, addr, section_gap):
        """
        Check if the address belongs to the address ranges.
        :param addr: checked address.
        :param section_gap: list of ranges.
        :return: True if the address belongs to any of ranges, False otherwise.
        """
        for l_addr in section_gap:
            start_addr = self.utils.to_int(l_addr[0])
            end_addr = start_addr + self.utils.to_int(l_addr[1])
            if self.utils.to_int(addr) >= start_addr and self.utils.to_int(addr) < end_addr:
                return True
        return False

    def get_symbols_by_section_name(self, symbols: List[Symbol],
                                    symbol_type: Literal["text", "data"]):
        """
        Returns list of symbols by name of section they belong to.
        :param symbols: list of symbols.
        :param symbol_type: substring as feature of the section name the symbol belongs to.
        :return: list of symbols.
        """
        return [symbol for symbol in symbols if symbol_type in symbol.section_name]

    def get_symbols(self) -> List[OrderedDict]:
        """"
        Transform kernel symbols list to address keyed symbols ordered dictionary.
        :return: ordered dictionary {sym_addr : Symbol()}.
        """
        if not self.__symbols:
            self.__symbols = {symbol.addr: symbol for symbol in self.__symbols_kernel_elf}
            self.__symbols = OrderedDict(sorted(self.__symbols.items()))

        return self.__symbols

    def get_relocs_text_rodata(self):
        """
        Derive relocations for rodata and text related sections.
        :return: lists [reloc_text1, reloc_text2, ..., reloc_textN],
                       [reloc_rodata1, reloc_rodata2, ..., reloc_rodataN].
        """
        relocs_text = []
        relocs_rodata = []

        relocs_str = self.get_raw_by_tool(self.__readelf_tool,
                                          ["-rW", "--elf-output-style=GNU",
                                          self.__elf_file])
        rel = re.compile(r"^(" + self.__re_hexadecimal + r")\s*", re.MULTILINE)

        section_text, section_rodata = self.get_rodata_text_scope()
        for el in rel.findall(relocs_str.strip()):
            rel_addr = self.utils.to_int(el)
            if self.filtered_addr_by_section(rel_addr, section_rodata):
                relocs_rodata.append(rel_addr)
            elif self.filtered_addr_by_section(rel_addr, section_text):
                relocs_text.append(rel_addr)
        relocs_text.sort()
        relocs_rodata.sort()
        return relocs_text, relocs_rodata

    def get_relocs_for_symbol(self, relocs_list, start_addr=None, end_addr=None):
        """"
        Derive relocations for addresses range.
        :param relocs_list: input relocation list.
        :param start_addr: start address :int.
        :param end_addr: end address: int.
        :return: [reloc1, reloc2, reloc3, ..., relocN].
        """
        ranged_rela = []
        if start_addr and end_addr is not None:
            for relocation in relocs_list:
                if self.utils.to_int(end_addr) <= self.utils.to_int(relocation):
                    break
                if self.utils.to_int(start_addr) <= self.utils.to_int(relocation):
                    ranged_rela.append(relocation)
        return ranged_rela

    def get_text_rodata_altinstructions_lists(self):
        """
        Derive full list of altinstructions in rodata and text related sections from ELF file.
        :return: [[text_alt_inst1_addr, length1], [text_alt_inst2_addr, length2], ...],
                 [[rodata_alt_inst1_addr, length1], [rodata_alt_inst2_addr, length2], ...].

        .altinstructions section contains an array of struct alt_instr.
        As instance, for kernel 4.14 from /arch/arm64/include/asm/alternative.h
        struct alt_instr {
            s32 orig_offset;    /* offset to original instruction */
            s32 alt_offset;     /* offset to replacement instruction */
            u16 cpufeature;     /* cpufeature bit set for replacement */
            u8  orig_len;       /* size of original instruction(s) */
            u8  alt_len;        /* size of new instruction(s), <= orig_len */
        };

        Later, address of original instruction can be calculated as
        at runtime     : &(alt_instr->orig_offset) + alt_instr->orig_offset + kernel offset
        ELF processing : address of .altinstruction section
                                    + in section offset of alt_instr structure
                                    + value of alt_instr.orig_offset
        details in /arch/arm64/kernel/alternative.c, void __apply_alternatives(void *, bool).
        """

        # The struct_format should reflect <struct alt_instr> content
        struct_format = "<iiHBB"
        pattern_altinst_section_content = "^ *0x[0-9A-Fa-f]{16} (.*) .*.{16}$"
        pattern_altinstr_section_addr = "^ *(0x[0-9A-Fa-f]{16}).*.*.{16}$"

        if self.__altinstr_text is not None:
            return self.__altinstr_text, self.__altinstr_rodata

        self.__altinstr_text = []
        self.__altinstr_rodata = []

        __hex_dump = self.get_raw_by_tool(self.__readelf_tool,
                                          ["--hex-dump=.altinstructions", self.__elf_file])
        if len(__hex_dump) == 0:
            return self.__altinstr_text, self.__altinstr_rodata

        # .altinstruction section start addr in ELF
        __altinstr_section_addr = int(re.findall(pattern_altinstr_section_addr,
                                                 __hex_dump, re.MULTILINE)[0], 16)

        # To provide .altinstruction section content using host readelf only
        # some magic with string parcing is needed
        hex_dump_list = re.findall(pattern_altinst_section_content, __hex_dump, re.MULTILINE)
        __hex_dump_str = "".join(hex_dump_list).replace(" ", "")
        __altinstr_section_bin = unhexlify(__hex_dump_str)
        __struct_size = struct.calcsize(struct_format)

        if (len(__altinstr_section_bin) % __struct_size) != 0:
            return self.__altinstr_text, self.__altinstr_rodata

        section_text, section_rodata = self.get_rodata_text_scope()
        if len(section_text) !=0 or len(section_rodata) !=0:
            __i = 0
            while __i < (len(__altinstr_section_bin) - __struct_size):
                __struct_byte = __altinstr_section_bin[__i: __i + __struct_size]
                __struct_value = list(struct.unpack(struct_format, __struct_byte))

                # original instruction addr (going to be replaced) considered as "gap"
                __original_instruction_addr = __struct_value[0] + __altinstr_section_addr + __i

                # derive the target ARM instruction(s) length.
                __target_instruction_len = __struct_value[3]

                if self.filtered_addr_by_section( __original_instruction_addr, section_text):
                    self.__altinstr_text.append([__original_instruction_addr,
                                                 __target_instruction_len])
                elif self.filtered_addr_by_section(__original_instruction_addr, section_rodata):
                    self.__altinstr_rodata.append([__original_instruction_addr,
                                                   __target_instruction_len])
                __i = __i + __struct_size
            self.__altinstr_text.sort()
            self.__altinstr_rodata.sort()
        return self.__altinstr_text, self.__altinstr_rodata

    def add_addrs_space_to_list(self, addr_list, addr_start, addr_end):
        """
        Add addresses by range into addresses list (addr_list).
        :param addr_list: target list where found ddresses are stored.
        :param addr_start: start of addresses range.
        :param addr_end: end of addresses range.
        """
        for addr in range(addr_start, addr_end):
            addr_list.append(addr)

    def get_altinstructions(self, alt_instr_list, start_addr=None, end_addr=None):
        """
        Derive list of altinstructions for address range.
        :param start_addr: start address range.
        :param end_addr: end address range.
        :param alt_instr_list: list of alternative instructions.
        :return: [[alt_inst1_addr, length1], [alt_inst2_addr, length2], ...].
        """
        ranged_altinst = []
        if len(alt_instr_list) == 0:
            return ranged_altinst
        if start_addr is not None and end_addr is not None:
            start_addr_int = self.utils.to_int(start_addr)
            end_addr_int = self.utils.to_int(end_addr)
            for l_instr in alt_instr_list:
                l_instr_addr_end = l_instr[0] + l_instr[1]
                if end_addr_int <= l_instr[0]:
                    break
                if (start_addr_int <= l_instr[0] < end_addr_int
                    and l_instr_addr_end < end_addr_int):
                    self.add_addrs_space_to_list(ranged_altinst, l_instr[0], l_instr_addr_end)
                elif (start_addr_int <= l_instr[0] < end_addr_int
                      and l_instr_addr_end >= end_addr_int):
                    self.add_addrs_space_to_list(ranged_altinst, l_instr[0], end_addr_int)
                elif start_addr_int > l_instr[0] and l_instr_addr_end < end_addr_int:
                    self.add_addrs_space_to_list(ranged_altinst, start_addr_int,
                                                 l_instr_addr_end)
                elif start_addr_int > l_instr[0] and l_instr_addr_end > end_addr_int:
                    self.add_addrs_space_to_list(ranged_altinst, start_addr_int, end_addr_int)
        return ranged_altinst

    def get_jump_table_list(self) -> list:
        """
        Return the list of jump table records in the whole kernel's ELF.
        :return: list of jump table records
                 [Sec_Jumptable_Data_rec1, Sec_Jumptable_Data_rec2, ...].
        """

        jump_table_start_sym = self.get_symbol_by_name(DEFAULT_NAME_JUMP_TABLE_START_SYM)
        jump_table_end_sym = self.get_symbol_by_name(DEFAULT_NAME_JUMP_TABLE_END_SYM)
        if jump_table_start_sym is None or jump_table_end_sym is None:
            return []

        jumptable_struct_size = struct.calcsize(self.jumptable_struct_format)
        jump_table_content = self.get_data_by_vaddr(jump_table_start_sym.addr,
                                                    jump_table_end_sym.addr
                                                    - jump_table_start_sym.addr)

        for i in range(ceil((jump_table_end_sym.addr
                             - jump_table_start_sym.addr)/jumptable_struct_size)):
            jtr = SecJumpTableData()
            begin = i * jumptable_struct_size
            end = begin + jumptable_struct_size

            (jtr.code,
            jtr.target_offset,
            jtr.key) = struct.unpack(self.jumptable_struct_format,
                                     jump_table_content[begin: end])

            jt_record_addr = jump_table_start_sym.addr + begin
            jtr.code += jt_record_addr
            jtr.target_offset += jt_record_addr
            self.__jt_rec.append(jtr)

        return self.__jt_rec

    def get_jump_table_module(self, start_addr: int, end_addr: int, jump_table: list) -> list:
        """
        Return JT related gaps are in range of our module.
        :param start_addr: address range start.
        :param end_addr: address range end.
        :param jump_table: full list (over whole kernel) of JT items.
        :return: list of addrs to be excluded [exclude_addr1, exclude_addr2, ...].
        """
        result_jt_gaps = []
        for jt_item in jump_table:
            if start_addr <= jt_item.code < end_addr:
                for __addr in range(jt_item.code, jt_item.code + DEFAULT_ARM_INST_WIDTH):
                    result_jt_gaps.append(__addr)
        return result_jt_gaps

    def get_symbol_by_name(self, sym_name: str) -> Symbol:
        """
        Return symbol by its name. Raises error if more then single function was found.
        :param sym_name: name of symbol.
        :return: symbol object or None.
        """
        found_symbols = list(filter(lambda s: s.name == sym_name, self.__symbols_kernel_elf))

        if len(found_symbols) > 1:
            raise ValueError(f"Was found several symbols for name {sym_name}.")

        return found_symbols[0] if found_symbols else None

    def get_symbol_by_vaddr(self, vaddrs=None):
        """
        Get symbol by virtual address.
        :param vaddrs: virtual address (int or list).
        :return: Symbol() or [Symbol()].
        """
        if isinstance(vaddrs, int):
            if vaddrs in self.get_symbols():
                return self.get_symbols()[vaddrs]
            for addr, symbol_obj in self.get_symbols().items():
                if (addr + symbol_obj.size) >= vaddrs >= addr:
                    return symbol_obj
        elif isinstance(vaddrs, list):
            symbol = [self.get_symbol_by_vaddr(vaddr) for vaddr in vaddrs]
            return symbol
        else:
            raise ValueError
        return None

    def get_section_by_name(self, sec_names=None):
        """
        Get section by_name.
        :param sec_names: section name (str or list).
        :return: Section() or [Section()].
        """
        if isinstance(sec_names, str):
            for _, section_obj in self.get_sections().items():
                if section_obj.name == sec_names:
                    return section_obj
        elif isinstance(sec_names, list):
            sections = [self.get_section_by_name(sec_name) for sec_name in sec_names]
            return sections
        else:
            raise ValueError
        return None

    def get_section_by_vaddr(self, vaddrs=None):
        """
        Get section by virtual address.
        :param vaddrs: virtual addresses (int or list).
        :return: Section() or [Section()].
        """
        if isinstance(vaddrs, int):
            if vaddrs in self.get_sections():
                return self.get_sections()[vaddrs]
            for addr, section_obj in self.get_sections().items():
                if (addr + section_obj.size) >= vaddrs >= addr:
                    return section_obj
        elif isinstance(vaddrs, list):
            sections = [self.get_symbol_by_vaddr(vaddr) for vaddr in vaddrs]
            return sections

        raise ValueError("Wrong format of the input virtual address.")


    def vaddr_to_file_offset(self, vaddrs):
        """
        Transform virtual address to file offset.
        :param vaddrs: virtual address (int or list).
        :return: file offset or list.
        """
        if isinstance(vaddrs, (str, int)):
            section = self.get_section_by_vaddr(vaddrs)
            return self.utils.to_int(vaddrs, 16) - section.addr + section.offset

        if isinstance(vaddrs, list):
            return [self.vaddr_to_file_offset(vaddr) for vaddr in vaddrs]

        raise ValueError("Wrong format of the input virtual address.")

    def get_data_by_vaddr(self, vaddr, size) -> bytearray:
        """
        Read content of ELF section by virtual address.
        :param vaddr: virtual address of area to be read.
        :param size: size of area to be read.
        :return: bytearray with area content.
        """
        with open(self.__elf_file, "rb") as elf_fp:
            elf_fp.seek(self.vaddr_to_file_offset(vaddr))
            outbuff = elf_fp.read(size)
        return outbuff
