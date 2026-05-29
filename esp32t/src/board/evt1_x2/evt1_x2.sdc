create_generated_clock -name xclk2 -source [get_ports {CLK_FPGA}] -master_clock exclk -divide_by 1 -multiply_by 4 [get_pins {u_Gowin_PLL/PLLA_inst/CLKOUT0}]
create_generated_clock -name pclk -source [get_ports {CLK_FPGA}] -master_clock exclk -divide_by 1 -multiply_by 1 [get_pins {u_Gowin_PLL/PLLA_inst/CLKOUT1}]
create_generated_clock -name hclk -source [get_ports {CLK_FPGA}] -master_clock exclk -divide_by 2 -multiply_by 1 [get_pins {u_Gowin_PLL/PLLA_inst/CLKOUT2}]
create_generated_clock -name gclk -source [get_ports {CLK_FPGA}] -master_clock exclk -divide_by 4 -multiply_by 1 [get_pins {u_Gowin_PLL/PLLA_inst/CLKOUT3}]
create_generated_clock -name xclk -source [get_ports {CLK_FPGA}] -master_clock exclk -divide_by 1 -multiply_by 2 [get_pins {u_Gowin_PLL/PLLA_inst/CLKOUT4}]

create_clock -name sclk -period 25 [get_ports {QSPI_CLK}]
create_clock -name exclk -period 29.802322 [get_ports {CLK_FPGA}]

set_clock_groups -asynchronous -group [get_clocks {pclk}] -group [get_clocks {hclk}]
set_clock_groups -asynchronous -group [get_clocks {pclk}] -group [get_clocks {gclk}]
set_clock_groups -asynchronous -group [get_clocks {hclk}] -group [get_clocks {gclk}]

set_max_delay -from [get_ports {CART_D[*]}] -to [get_clocks {hclk}] 13
set_max_delay -from [get_clocks {hclk}] -to  [get_ports {CART_A[*]}] 14
set_max_delay -from [get_clocks {hclk}] -to  [get_ports {CART_WR}] 14
set_max_delay -from [get_clocks {hclk}] -to  [get_ports {CART_RD}] 14
set_max_delay -from [get_clocks {hclk}] -to  [get_ports {CART_CS}] 14
set_max_delay -from [get_clocks {hclk}] -to  [get_ports {LINK_SD}] 14
set_max_delay -from [get_clocks {hclk}] -to  [get_ports {CART_D[*]}] 14

create_clock -name ck24 -period 41.666667 -waveform {0 20.833333} [get_ports {CLK_24MHz}]

set_clock_groups -asynchronous -group [get_clocks {pclk}] -group [get_clocks {hclk}]
set_clock_groups -asynchronous -group [get_clocks {pclk}] -group [get_clocks {gclk}]
set_clock_groups -asynchronous -group [get_clocks {hclk}] -group [get_clocks {gclk}]

// USB Clocks
create_generated_clock -name PHY_CLKOUT -source [get_ports {CLK_24MHz}] -master_clock ck24 -divide_by 16 -multiply_by 40 [get_pins {u_usb_top/u_Gowin_PLL_USB/PLLA_inst/CLKOUT1}]
create_generated_clock -name fclk_960M -source [get_ports {CLK_24MHz}] -master_clock ck24 -divide_by 1 -multiply_by 40 [get_nets {u_usb_top/fclk_960M}]
create_generated_clock -name clk24p -source [get_ports {CLK_24MHz}] -master_clock ck24 -divide_by 1 -multiply_by 1 [get_pins {u_usb_top/u_Gowin_PLL_USB/PLLA_inst/CLKOUT2}]
create_clock -name usbintsclk -period 8 -waveform {0 4} [get_nets {u_usb_top/u_USB_SoftPHY_Top/usb2_0_softphy/u_usb_20_phy_utmi/u_usb2_0_softphy/u_usb_phy_hs/sclk}] -add
set_clock_groups -asynchronous -group [get_clocks {PHY_CLKOUT}] -group [get_clocks {fclk_960M}]
set_clock_groups -asynchronous -group [get_clocks {PHY_CLKOUT}] -group [get_clocks {usbintsclk}]

///// FlashGBX LK /////

// Explicit CDC
set_clock_groups -asynchronous -group [get_clocks {PHY_CLKOUT}] -group [get_clocks {xclk}]
set_false_path -to [get_regs {*/lk_cdc_*0_s0}]
// Copies within `top` to decouple from PHY
set_false_path -from [get_regs {lk_enabled_d*}]

set_multicycle_path -setup 2 -from [get_regs {u_lk/u_core/*}] -to   [get_regs {u_lk/rx_valid_o* u_lk/rx_data_o* u_lk/tx_valid* u_lk/tx_data* u_lk/cart_complete*sr*}]
set_multicycle_path -hold 1  -from [get_regs {u_lk/u_core/*}] -to   [get_regs {u_lk/rx_valid_o* u_lk/rx_data_o* u_lk/tx_valid* u_lk/tx_data* u_lk/cart_complete*sr*}]
set_multicycle_path -setup 2 -to   [get_regs {u_lk/u_core/*}] -from [get_regs {u_lk/rx_valid_o* u_lk/rx_data_o* u_lk/tx_valid* u_lk/tx_data* u_lk/cart_complete*sr*}]
set_multicycle_path -hold 1  -to   [get_regs {u_lk/u_core/*}] -from [get_regs {u_lk/rx_valid_o* u_lk/rx_data_o* u_lk/tx_valid* u_lk/tx_data* u_lk/cart_complete*sr*}]

set_multicycle_path -setup 2 -to   [get_regs {u_lk/u_core/*}] -from [get_regs {u_lk/cart_complete*sr*}]
set_multicycle_path -hold 1  -to   [get_regs {u_lk/u_core/*}] -from [get_regs {u_lk/cart_complete*sr*}]

// FIFO can be distant from u_lk/u_core
set_multicycle_path -setup 2 -from [get_regs {u_lk/req_enqueue*}] -to [get_regs {u_lk/u_cart_req_fifo/*}]
set_multicycle_path -hold 1  -from [get_regs {u_lk/req_enqueue*}] -to [get_regs {u_lk/u_cart_req_fifo/*}]
// FIFO can be distant from u_lk
set_multicycle_path -setup 2 -from [get_regs {u_lk/cart_req_valid_d* u_lk/cart_req_d*}] -to [get_regs {u_lk/u_cart_executor/*}]
set_multicycle_path -hold 1  -from [get_regs {u_lk/cart_req_valid_d* u_lk/cart_req_d*}] -to [get_regs {u_lk/u_cart_executor/*}]

// Correct (loosen) the timing requirements for resetting the DRAM.
//
// Not logically needed for FlashGBX LK, but the added complexity makes the routing harder
set_multicycle_path -from [get_clocks {xclk}] -to [get_clocks {xclk2}] -setup 2
set_multicycle_path -from [get_clocks {xclk}] -to [get_clocks {xclk2}] -hold 1

report_timing -setup -max_paths 25
report_timing -hold -max_paths 25
report_timing -recovery -max_paths 25
report_timing -removal -max_paths 25

report_timing -setup -max_paths 25 -max_common_paths 1 -mod_ins {u_lk}
report_timing -hold -max_paths 25 -max_common_paths 1 -mod_ins {u_lk}
report_timing -recovery -max_paths 25 -max_common_paths 1 -mod_ins {u_lk}
report_timing -removal -max_paths 25 -max_common_paths 1 -mod_ins {u_lk}
