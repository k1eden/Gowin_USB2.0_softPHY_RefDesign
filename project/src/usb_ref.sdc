//Copyright (C)2014-2021 GOWIN Semiconductor Corporation.
//All rights reserved.
//File Title: Timing Constraints file
//GOWIN Version: 1.9.8.02 
//Created Time: 2021-12-30 10:39:51
create_clock -name clkout -period 2.083 -waveform {0 1.042} [get_nets {fclk_480M}] -add
create_clock -name sclk -period 8.333 -waveform {0 4.167} [get_nets {u_USB_SoftPHY_Top/u_usb2_0_softphy/u_usb_phy_hs/sclk}] -add
create_clock -name clkin -period 83.333 -waveform {0 41.666} [get_ports {CLK_IN}] -add
create_clock -name PHY_CLKOUT -period 16.666 -waveform {0 8.197} [get_nets {PHY_CLKOUT}] -add
set_clock_groups -asynchronous -group [get_clocks {PHY_CLKOUT}] -group [get_clocks {sclk}]
