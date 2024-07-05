-- #################################################################################################
-- # << The NEORV32 RISC-V Processor - LiteX NEORV32 Core Complex Wrapper >>                       #
-- # ********************************************************************************************* #
-- #                                      __   _ __      _  __                                     #
-- #                                     / /  (_) /____ | |/_/                                     #
-- #                                    / /__/ / __/ -_)>  <                                       #
-- #                                   /____/_/\__/\__/_/|_|                                       #
-- #                                 Build your hardware, easily!                                  #
-- #                                                                                               #
-- # Unless otherwise noted, LiteX is copyright (C) 2012-2022 Enjoy-Digital. All rights reserved.  #
-- # LiteX HQ: https://github.com/enjoy-digital/litex                                              #
-- #                                                                                               #
-- # ********************************************************************************************* #
-- # NEORV32 Core Complex wrapper for the LiteX SoC builder framework.                             #
-- # https://github.com/enjoy-digital/litex/tree/master/litex/soc/cores/cpu/neorv32                #
-- #                                                                                               #
-- # This wrapper provides four pre-configured core complex configurations: "minimal", "lite",     #
-- # "standard" and "full". See the 'configs_c' table for more details which RISC-V ISA extensions #
-- # and module parameters are used by each of the these configurations. All configurations can be #
-- # used with the RISC-V-compatible on-chip debugger ("DEBUG").                                   #
-- #                                                                                               #
-- # === Bus Interface ===                                                                         #
-- # This wrappers uses the "pipelined" Wishbone b4 protocol for the bus interface. See the        #
-- # "global configuration" constants for further bus configuration parameters (endianness,        #
-- # timeout, etc.).                                                                               #
-- #                                                                                               #
-- # === Interrupt ====                                                                            #
-- # The external interrupt signal is delegated to the CPU as RISC-V "machine external interrupt   #
-- # (MTI)". Note that this IRQ signal is high-active - once set the signal has to stay high until #
-- # the interrupt request is explicitly acknowledged (e.g. writing to a memory-mapped register)!  #
-- #                                                                                               #
-- # === Core Complex Address Space ===                                                            #
-- # Note that the NEORV32 core complex occupies a small fraction of the total 32-bit address      #
-- # space for internal components (machine timer, on-chip-debugger, ...). This address space      #
-- # starts at address 0xffff0000 and ends at 0xffffffff. Any CPU access to this address space     #
-- # will NOT be delegated to bus interface of the core complex!                                   #
-- # ********************************************************************************************* #
-- # BSD 3-Clause License                                                                          #
-- #                                                                                               #
-- # Copyright (c) 2023, Stephan Nolting. All rights reserved.                                     #
-- #                                                                                               #
-- # Redistribution and use in source and binary forms, with or without modification, are          #
-- # permitted provided that the following conditions are met:                                     #
-- #                                                                                               #
-- # 1. Redistributions of source code must retain the above copyright notice, this list of        #
-- #    conditions and the following disclaimer.                                                   #
-- #                                                                                               #
-- # 2. Redistributions in binary form must reproduce the above copyright notice, this list of     #
-- #    conditions and the following disclaimer in the documentation and/or other materials        #
-- #    provided with the distribution.                                                            #
-- #                                                                                               #
-- # 3. Neither the name of the copyright holder nor the names of its contributors may be used to  #
-- #    endorse or promote products derived from this software without specific prior written      #
-- #    permission.                                                                                #
-- #                                                                                               #
-- # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS   #
-- # OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF               #
-- # MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE    #
-- # COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,     #
-- # EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE #
-- # GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED    #
-- # AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING     #
-- # NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED  #
-- # OF THE POSSIBILITY OF SUCH DAMAGE.                                                            #
-- # ********************************************************************************************* #
-- # The NEORV32 Processor - https://github.com/stnolting/neorv32              (c) Stephan Nolting #
-- #################################################################################################

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library neorv32;
use neorv32.neorv32_package.all;

entity neorv32_litex_core_complex is
  generic (
    CONFIG : natural; -- configuration select (0=minimal, 1=lite, 2=standard, 3=full)
    DEBUG  : boolean  -- enable on-chip debugger, valid for all configurations
  );
  port (
    -- Global control --
    clk_i       : in  std_ulogic; -- global clock, rising edge
    rstn_i      : in  std_ulogic; -- global reset, low-active, async

    -- JTAG on-chip debugger interface --
    jtag_trst_i : in  std_ulogic; -- low-active TAP reset (optional)
    jtag_tck_i  : in  std_ulogic; -- serial clock
    jtag_tdi_i  : in  std_ulogic; -- serial data input
    jtag_tdo_o  : out std_ulogic; -- serial data output
    jtag_tms_i  : in  std_ulogic; -- mode select

    -- External bus interface (Wishbone) --
    wb_adr_o    : out std_ulogic_vector(31 downto 0); -- address
    wb_dat_i    : in  std_ulogic_vector(31 downto 0); -- read data
    wb_dat_o    : out std_ulogic_vector(31 downto 0); -- write data
    wb_we_o     : out std_ulogic; -- read/write
    wb_sel_o    : out std_ulogic_vector(3 downto 0); -- byte enable
    wb_stb_o    : out std_ulogic; -- strobe
    wb_cyc_o    : out std_ulogic; -- valid cycle
    wb_ack_i    : in  std_ulogic; -- transfer acknowledge
    wb_err_i    : in  std_ulogic; -- transfer error

    -- CPU interrupt --
    mext_irq_i  : in  std_ulogic  -- RISC-V machine external interrupt (MEI)
  );
end neorv32_litex_core_complex;

architecture neorv32_litex_core_complex_rtl of neorv32_litex_core_complex is

  -- identifiers --
  constant hart_id_c  : std_ulogic_vector(31 downto 0) := x"00000000"; -- hardware thread ID ("core ID")
  constant jedec_id_c : std_ulogic_vector(10 downto 0) := "00000000000"; -- vendor's JEDEC manufacturer ID

  -- advanced configuration --
  constant num_configs_c : natural := 4;    -- number of pre-defined configurations
  constant wb_timeout_c  : natural := 1024; -- external bus interface timeout cycles

  -- helpers --
  type bool_t is array (0 to num_configs_c-1) of boolean;
  type natural_t is array (0 to num_configs_c-1) of natural;
  type configs_t is record
    riscv_c      : bool_t;
    riscv_m      : bool_t;
    riscv_u      : bool_t;
    riscv_zicntr : bool_t;
    riscv_zihpm  : bool_t;
    fast_ops     : bool_t;
    pmp_num      : natural_t;
    hpm_num      : natural_t;
    xcache_en    : bool_t;
    xcache_nb    : natural_t;
    xcache_bs    : natural_t;
    mtime        : bool_t;
  end record;

  -- core complex configurations --
  constant configs_c : configs_t := (
    --               minimal   lite    standard  full
    riscv_c      => ( false,   true,    true,    true  ), -- RISC-V compressed instructions 'C'
    riscv_m      => ( false,   true,    true,    true  ), -- RISC-V hardware mul/div 'M'
    riscv_u      => ( false,   false,   true,    true  ), -- RISC-V user mode 'U'
    riscv_zicntr => ( false,   false,   true,    true  ), -- RISC-V standard CPU counters 'Zicntr'
    riscv_zihpm  => ( false,   false,   false,   true  ), -- RISC-V hardware performance monitors 'Zihpm'
    fast_ops     => ( false,   false,   true,    true  ), -- use DSPs and barrel-shifters
    pmp_num      => ( 0,       0,       0,       8     ), -- number of PMP regions (0..16)
    hpm_num      => ( 0,       0,       0,       8     ), -- number of HPM counters (0..29)
    xcache_en    => ( false,   false,   true,    true  ), -- external bus cache enabled
    xcache_nb    => ( 0,       0,       32,      64    ), -- number of cache blocks (lines), power of two
    xcache_bs    => ( 0,       0,       32,      32    ), -- size of cache clock (lines) in bytes, power of two
    mtime        => ( false,   true,    true,    true  )  -- RISC-V machine system timer
  );

begin

  -- NEORV32 Core Complex -------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_core_complex: neorv32_top
  generic map (
    -- General --
    CLOCK_FREQUENCY            => 0,                              -- clock frequency of clk_i in Hz [not required by the core complex]
    HART_ID                    => hart_id_c,                      -- hardware thread ID
    JEDEC_ID                   => jedec_id_c,                     -- vendor's JEDEC ID
    -- On-Chip Debugger (OCD) --
    ON_CHIP_DEBUGGER_EN        => DEBUG,                          -- implement on-chip debugger
    -- RISC-V CPU Extensions --
    CPU_EXTENSION_RISCV_C      => configs_c.riscv_c(CONFIG),      -- implement compressed extension?
    CPU_EXTENSION_RISCV_M      => configs_c.riscv_m(CONFIG),      -- implement mul/div extension?
    CPU_EXTENSION_RISCV_U      => configs_c.riscv_u(CONFIG),      -- implement user mode extension?
    CPU_EXTENSION_RISCV_Zicntr => configs_c.riscv_zicntr(CONFIG), -- implement base counters?
    CPU_EXTENSION_RISCV_Zihpm  => configs_c.riscv_zihpm(CONFIG),  -- implement hardware performance monitors?
    -- Tuning Options --
    FAST_MUL_EN                => configs_c.fast_ops(CONFIG),     -- use DSPs for M extension's multiplier
    FAST_SHIFT_EN              => configs_c.fast_ops(CONFIG),     -- use barrel shifter for shift operations
    -- Physical Memory Protection (PMP) --
    PMP_NUM_REGIONS            => configs_c.pmp_num(CONFIG),      -- number of regions (0..16)
    PMP_MIN_GRANULARITY        => 4,                              -- minimal region granularity in bytes, has to be a power of 2, min 4 bytes
    -- Hardware Performance Monitors (HPM) --
    HPM_NUM_CNTS               => configs_c.hpm_num(CONFIG),      -- number of implemented HPM counters (0..29)
    HPM_CNT_WIDTH              => 64,                             -- total size of HPM counters (0..64)
    -- External bus interface (XBUS) --
    XBUS_EN                    => true,                           -- implement external memory bus interface?
    XBUS_TIMEOUT               => wb_timeout_c,                   -- cycles after a pending bus access auto-terminates (0 = disabled)
    XBUS_PIPE_MODE             => false,                          -- protocol: false=classic/standard wishbone mode, true=pipelined wishbone mode
    XBUS_ASYNC_RX              => true,                           -- use register buffer for RX data when false
    XBUS_ASYNC_TX              => true,                           -- use register buffer for TX data when false
    XBUS_CACHE_EN              => configs_c.xcache_en(CONFIG),    -- enable external bus cache (x-cache)
    XBUS_CACHE_NUM_BLOCKS      => configs_c.xcache_nb(CONFIG),    -- x-cache: number of blocks (min 1), has to be a power of 2
    XBUS_CACHE_BLOCK_SIZE      => configs_c.xcache_bs(CONFIG),    -- x-cache: block size in bytes (min 4), has to be a power of 2
    -- Processor peripherals --
    IO_MTIME_EN                => configs_c.mtime(CONFIG)         -- implement machine system timer (MTIME)?
  )
  port map (
    -- Global control --
    clk_i       => clk_i,       -- global clock, rising edge
    rstn_i      => rstn_i,      -- global reset, low-active, async
    -- JTAG on-chip debugger interface --
    jtag_trst_i => jtag_trst_i, -- low-active TAP reset (optional)
    jtag_tck_i  => jtag_tck_i,  -- serial clock
    jtag_tdi_i  => jtag_tdi_i,  -- serial data input
    jtag_tdo_o  => jtag_tdo_o,  -- serial data output
    jtag_tms_i  => jtag_tms_i,  -- mode select
    -- External bus interface --
    xbus_adr_o  => wb_adr_o,    -- address
    xbus_dat_i  => wb_dat_i,    -- read data
    xbus_dat_o  => wb_dat_o,    -- write data
    xbus_we_o   => wb_we_o,     -- read/write
    xbus_sel_o  => wb_sel_o,    -- byte enable
    xbus_stb_o  => wb_stb_o,    -- strobe
    xbus_cyc_o  => wb_cyc_o,    -- valid cycle
    xbus_ack_i  => wb_ack_i,    -- transfer acknowledge
    xbus_err_i  => wb_err_i,    -- transfer error
    -- CPU Interrupts --
    mext_irq_i  => mext_irq_i   -- machine external interrupt
  );


end neorv32_litex_core_complex_rtl;
