quit -sim

vlib work  

#mapping work library to current directory
#vmap [-help] [-c] [-del] [<logical_name>] [<path>]
vmap work  

#compile all .v files to work library
#-work <path>       Specify library WORK
#-vlog01compat      Ensure compatibility with Std 1364-2001
#-incr              Enable incremental compilation
#"rtl/*.v"          rtl directory all .v files, support relative path, need to add ""
#vlog
vlog -work work -vlog01compat -incr "./../../tb/prim_sim.v"
vlog -work work -vlog01compat -incr "./../../tb/prim_sim_h.v"
vlog -work work -vlog01compat -incr "./../../tb/usb_host/usbh_crc5.v"
vlog -work work -vlog01compat -incr "./../../tb/usb_host/usbh_crc16.v"
vlog -work work -vlog01compat -incr "./../../tb/usb_host/usbh_fifo.v"
vlog -work work -vlog01compat -incr "./../../tb/usb_host/usbh_host.v"
vlog -work work -vlog01compat -incr "./../../tb/usb_host/usbh_host_defs.v"
vlog -work work -vlog01compat -incr "./../../tb/usb_host/usbh_sie.v"
vlog -work work -vlog01compat -incr "./../../tb/usb_host/usb_define.v"
vlog -work work -vlog01compat -incr "./../../project/src/TOP.v"
vlog -work work -vlog01compat -incr "./../../project/src/gowin_pllvr/gowin_pllvr.v"
vlog -work work -vlog01compat -incr "./../../project/src/fifo_sc_top/fifo_sc_top.vo"
vlog -work work -vlog01compat -incr "./../../project/src/usb_descriptor.v"
vlog -work work -vlog01compat -incr "./../../project/src/usb2_0_softphy/usb2_0_softphy.vo"
vlog -work work -vlog01compat -incr "./../../project/src/usb_device_controller/usb_device_controller.vo"
vlog -work work -vlog01compat -incr "./../../tb/tb.v"


#complie all .vhd files
#-work <path>       Specify library WORK
#-93                Enable support for VHDL 1076-1993
#-2002              Enable support for VHDL 1076-2002
#vcom


#simulate testbench top file
#-L <libname>                     Search library for design units instantiated from Verilog and for VHDL default component binding
#+nowarn<CODE | Number>           Disable specified warning message  (Example: +nowarnTFMPC)                      
#-t [1|10|100]fs|ps|ns|us|ms|sec  Time resolution limit VHDL default: resolution setting from .ini file) 
#                                 (Verilog default: minimum time_precision in the design)
#-novopt                          Force incremental mode (pre-6.0 behavior)
vsim +nowarnTFMPC -L work  -novopt -l tb.log work.tb

#generate wave log format(WLF)......
log -r /*

#open wave window
view wave

#add simulation singals
do wave.do

#set simulation time
run  -all



