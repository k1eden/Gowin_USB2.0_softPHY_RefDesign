onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate /tb/u_sie/utmi_txvalid_r
add wave -noupdate /tb/u_sie/utmi_txvalid_o
add wave -noupdate /tb/u_sie/utmi_txready_i
add wave -noupdate /tb/u_sie/utmi_data_o
add wave -noupdate /tb/u_Top/u_usb_device_controller_top/utmi_rxvalid_i
add wave -noupdate /tb/u_Top/u_usb_device_controller_top/utmi_rxerror_i
add wave -noupdate /tb/u_Top/u_usb_device_controller_top/utmi_rxactive_i
add wave -noupdate /tb/u_Top/u_usb_device_controller_top/utmi_datain_i
add wave -noupdate /tb/u_Top/u_USB_SoftPHY_Top/fclk_i
add wave -noupdate /tb/u_Top/u_USB_SoftPHY_Top/clk_i
add wave -noupdate /tb/u_Top/u_USB_SoftPHY_Top/usb_rxdp_i
add wave -noupdate /tb/u_Top/u_USB_SoftPHY_Top/usb_rxdn_i
add wave -noupdate /tb/u_USB_SoftPHY_Top/utmi_txvalid_i
add wave -noupdate /tb/u_USB_SoftPHY_Top/utmi_txready_o
add wave -noupdate /tb/u_USB_SoftPHY_Top/utmi_termselect_i
add wave -noupdate /tb/u_USB_SoftPHY_Top/utmi_rxvalid_o
add wave -noupdate /tb/u_USB_SoftPHY_Top/utmi_rxactive_o
add wave -noupdate /tb/u_USB_SoftPHY_Top/utmi_data_out_i
add wave -noupdate /tb/u_USB_SoftPHY_Top/usb_dxp_io
add wave -noupdate /tb/u_USB_SoftPHY_Top/usb_dxn_io
add wave -noupdate {/tb/u_Top/u_USB_SoftPHY_Top/\u_usb2_0_softphy/u_usb_phy_hs/i_rx_phy/u_cdr_serdes_x8_dx/rx_data }
add wave -noupdate /tb/u_Top/u_USB_SoftPHY_Top/utmi_rxvalid_o
add wave -noupdate /tb/u_Top/u_USB_SoftPHY_Top/utmi_rxerror_o
add wave -noupdate /tb/u_Top/u_USB_SoftPHY_Top/utmi_rxactive_o
add wave -noupdate /tb/u_Top/u_USB_SoftPHY_Top/utmi_data_in_o
add wave -noupdate /tb/u_Top/PHY_RESET
add wave -noupdate /tb/u_Top/u_USB_SoftPHY_Top/q_nrzi_data
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {120062309 ps} 0}
quietly wave cursor active 1
configure wave -namecolwidth 636
configure wave -valuecolwidth 100
configure wave -justifyvalue left
configure wave -signalnamewidth 0
configure wave -snapdistance 10
configure wave -datasetprefix 0
configure wave -rowmargin 4
configure wave -childrowmargin 2
configure wave -gridoffset 0
configure wave -gridperiod 1
configure wave -griddelta 40
configure wave -timeline 0
configure wave -timelineunits ns
update
WaveRestoreZoom {120045440 ps} {120090560 ps}
