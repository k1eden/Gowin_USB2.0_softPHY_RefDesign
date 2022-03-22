
//===========================================
`timescale 1ns/1ps

`include "./usb_host/usb_define.v"
module tb;

GSR GSR (.GSRI(1));
reg  reset;
reg  clk_in;
wire clk_60;
wire [7:0] host_utmi_data_out  ;
wire       host_utmi_txvalid   ;
wire [1:0] host_utmi_op_mode   ;
wire [1:0] host_utmi_xcvrselect;
wire       host_utmi_termselect;
wire       host_utmi_dppulldown;
wire       host_utmi_dmpulldown;
wire [7:0] host_utmi_data_in   ;
wire       host_utmi_txready   ;
wire       host_utmi_rxvalid   ;
wire       host_utmi_rxactive  ;
wire       host_utmi_rxerror   ;
wire [1:0] host_utmi_linestate ;
reg [15:0] data_len;
reg [7:0]  token_pid;
reg [6:0]  token_dev;
reg [3:0]  token_ep;
reg        transfer_start_q;
reg        sof_transfer_q;
wire       tx_pop;
reg [7:0]  tx_data;
reg [7:0]  set_addr [0:7];
reg [7:0]  set_line_coding [0:14];
reg [7:0]  line_coding_data [0:6];
reg [7:0]  set_interface [0:7];
reg [7:0]  get_line_coding [0:7];
reg [7:0]  test_mode_data [0:7];
reg [7:0]  send_data [0:38];
reg        in_transfer;
wire       rx_push;
wire [7:0] rx_data;
wire       rx_done;
wire       tx_done;
integer    i;
integer    e;
reg [7:0] byte_cnt = 0;
reg [7:0] color_sel = 0;

//==============================================================
//======Clock 48MHz
initial begin
    clk_in= 0;
    reset = 1;
#100000
    reset = 0;
end
always #10 clk_in <= ~clk_in;

Gowin_PLLVR u_pll(
    .clkout (fclk_480M ), //output clkout
    .clkoutd(clk_60    ), //output clkout
    .lock   (pll_locked),
    .clkin  (clk_in    )  //input clkin
);
//==============================================================
//======USB Host Controller
initial begin
    transfer_start_q <= 1'b0;
    sof_transfer_q   <= 1'b0;
    in_transfer <= 1'b0;
    data_len = 16'd8;
#120000
//======Set USB Device Addr
    wait(!reset);
    usb_host_send(`PID_SETUP, 7'd0, 4'd0);
    data_len = 16'd8;
    for(i=0;i<8;i=i+1) begin
        tx_data = set_addr[i];
        wait(tx_pop);
        @(posedge clk_60);
    end
#2000
        in_transfer = 1'b1;
        usb_host_send(`PID_IN, 7'h0, 4'd0);
        #160000;
#8000
//======Setup USB CDC Set Interface
    in_transfer = 1'b0;
#8000
    data_len = 16'd8;
    usb_host_send(`PID_SETUP, 7'h40, 4'd0);
    for(i=0;i<8;i=i+1) begin
        tx_data = set_interface[i];
        wait(tx_pop);
        @(posedge clk_60);
    end
    $display("\n Set Interface ");
#8000
//======Setup USB CDC Line Coding
    in_transfer = 1'b0;
    data_len = 16'd8;
    usb_host_send(`PID_SETUP, 7'h40, 4'd0);
    for(i=0;i<8;i=i+1) begin
        tx_data = set_line_coding[i];
        wait(tx_pop);
        @(posedge clk_60);
    end
    $display("\n Set Line Coding ");
#8000
//======Send USB CDC Line Coding Data
    in_transfer = 1'b0;
    data_len = 16'd7;
    usb_host_send(`PID_OUT, 7'h40, 4'd0);
    for(i=0;i<7;i=i+1) begin
        tx_data = line_coding_data[i];
        wait(tx_pop);
        @(posedge clk_60);
    end
    $display("\n Set Line Coding Data");
#8000
//======Send USB CDC Line Coding Data
    in_transfer = 1'b0;
    data_len = 16'd8;
    usb_host_send(`PID_SETUP, 7'h40, 4'd0);
    for(i=0;i<8;i=i+1) begin
        tx_data = get_line_coding[i];
        wait(tx_pop);
        @(posedge clk_60);
    end
    $display("\n Get Line Coding Data");
#8000
    in_transfer = 1'b1;
    usb_host_send(`PID_IN, 7'h40, 4'd0);
    i=0;
    while(!rx_done) begin
            wait(rx_push);
            i=i+1;
            @(posedge clk_60);
    end
    $display("\n Get Line Coding Data Done");
#8000
#8000
////======Send USB UART Test Data
//    in_transfer = 1'b0;
//    data_len = 16'd38;
//    usb_host_send(`PID_OUT, 7'h40, 4'd2);
//    for(i=0;i<38;i=i+1) begin
//        tx_data = send_data[i];
//        wait(tx_pop);
//        @(posedge clk_60);
//        @(posedge clk_60);
//    end
//    $display("\n Set UART Test Data");
//#900000
//    //Get UART Test Data from Endpoint 2 IN
//    in_transfer = 1'b1;
//    usb_host_send(`PID_IN, 7'h40, 4'd2);
//    i=0;
//    while(!rx_done) begin
//            wait(rx_push);
//            if (rx_data == send_data[i]) begin
//                $display("\n RX Data %d == TX Data %d",rx_data,send_data[i]);
//            end
//            else begin
//                $display("\n RX Data %d == TX Data %d",rx_data,send_data[i]);
//                $display("\n IN/OUT test failed");
//                //$stop;
//            end
//            i=i+1;
//            @(posedge clk_60);
//            @(posedge clk_60);
//    end
//    $display("\n IN/OUT test passed");
#8000
#180000
$stop;
end

initial begin
    test_mode_data[0] = 8'h00;
    test_mode_data[1] = 8'h03;//set feature
    test_mode_data[2] = 8'h02;//feature selector
    test_mode_data[3] = 8'h00;//feature selector
    test_mode_data[4] = 8'h00;//Zero Interface Endpoint
    //test_mode_data[5] = 8'h01;//Test Selector J
    //test_mode_data[5] = 8'h02;//Test Selector K
    //test_mode_data[5] = 8'h03;//Test Selector SE0
    test_mode_data[5] = 8'h04;//Test Selector Packet
    //test_mode_data[5] = 8'h05;//Test Selector Force Enable for hub only
    test_mode_data[6] = 8'h00;//wLength
    test_mode_data[7] = 8'h00;//wLength
end
initial begin
    set_addr[0] = 8'h00;
    set_addr[1] = 8'h05;
    set_addr[2] = 8'h01;
    set_addr[3] = 8'h00;
    set_addr[4] = 8'h00;
    set_addr[5] = 8'h00;
    set_addr[6] = 8'h00;
    set_addr[7] = 8'h00;
end
initial begin
    set_line_coding[0] = 8'h21;
    set_line_coding[1] = 8'h20;
    set_line_coding[2] = 8'h00;
    set_line_coding[3] = 8'h00;
    set_line_coding[4] = 8'h00;
    set_line_coding[5] = 8'h00;
    set_line_coding[6] = 8'h07;
    set_line_coding[7] = 8'h00;
end
initial begin
    set_interface[0] = 8'h01;
    set_interface[1] = 8'h0b;
    set_interface[2] = 8'h00;
    set_interface[3] = 8'h00;
    set_interface[4] = 8'h01;
    set_interface[5] = 8'h00;
    set_interface[6] = 8'h00;
    set_interface[7] = 8'h00;
end
initial begin
    line_coding_data[0] = 8'h00;//Baud Rate LSB
    line_coding_data[1] = 8'hC2;//Baud Rate
    line_coding_data[2] = 8'h01;//Baud Rate
    line_coding_data[3] = 8'h00;//Baud Rate MSB
    line_coding_data[4] = 8'h00;//CharFormat 0-1 Stop bit 1-1.5 Stop bits 2-2 Stop bits
    line_coding_data[5] = 8'h00;//Parity 0-None 1-Odd 2-Even 3-Mark 4-Space
    line_coding_data[6] = 8'h08;//Data bits(5 6 7 8)
end
initial begin
    get_line_coding[0] = 8'hA1;
    get_line_coding[1] = 8'h21;
    get_line_coding[2] = 8'h00;
    get_line_coding[3] = 8'h00;
    get_line_coding[4] = 8'h00;
    get_line_coding[5] = 8'h00;
    get_line_coding[6] = 8'h07;
    get_line_coding[7] = 8'h00;
end

initial begin
    send_data[0]  = 8'h0C;
    send_data[1]  = 8'h0C;
    send_data[2]  = 8'hFF;
    send_data[3]  = 8'hFF;
    send_data[4]  = 8'hFF;
    send_data[5]  = 8'hFF;
    send_data[6]  = 8'hFF;
    send_data[7]  = 8'hFF;
    send_data[8]  = 8'hFF;
    send_data[9]  = 8'hFF;
    send_data[10] = 8'hFF;
    send_data[11] = 8'hFF;
    send_data[12] = 8'hFF;
    send_data[13] = 8'hFF;
    send_data[14] = 8'hFF;
    send_data[15] = 8'hFF;
    send_data[16] = 8'hFF;
    send_data[17] = 8'hFF;
    send_data[18] = 8'hFF;
    send_data[19] = 8'hFF;
    send_data[20] = 8'h00;
    send_data[21] = 8'h00;
    send_data[22] = 8'h00;
    send_data[23] = 8'h00;
    send_data[24] = 8'h00;
    send_data[25] = 8'h00;
    send_data[26] = 8'h00;
    send_data[27] = 8'h00;
    send_data[28] = 8'd0;
    send_data[29] = 8'd108;
    send_data[30] = 8'd6;
    send_data[31] = 8'd0;
    send_data[32] = 8'd92;
    send_data[33] = 8'd0;
    send_data[34] = 8'hFF;
    send_data[35] = 8'hFF;
    send_data[36] = 8'hFF;
    send_data[37] = 8'hFF;
end

task usb_host_send;
    input [7:0] pid;
    input [6:0] dev;
    input [3:0] ep;
    begin

    token_pid = pid;
    token_dev = dev;
    token_ep = {ep[0],ep[1],ep[2],ep[3]};
    $display("\n  pid = %h  dev = %h  ep=%h", pid, dev, ep);
    transfer_start_q <= 1'b1;
#20
    transfer_start_q <= 1'b0;
    end

endtask
usbh_sie
#( .USB_CLK_FREQ(60000000) )
u_sie
(
    // Clock & reset
    .clk_i(clk_60),
    .rst_i(reset),
    // Control
    .start_i(transfer_start_q),
    .in_transfer_i(in_transfer),
    .sof_transfer_i(sof_transfer_q),
    .resp_expected_i(1'b0),
    .ack_o(),//transfer_ack_w
    // Token packet
    .token_pid_i(token_pid),
    .token_dev_i(token_dev),
    .token_ep_i (token_ep),
    // Data packet
    .data_len_i(data_len),
    .data_idx_i(0),
    // Tx Data FIFO
    .tx_data_i(tx_data),
    .tx_pop_o(tx_pop),
    // Rx Data FIFO
    .rx_data_o(rx_data),
    .rx_push_o(rx_push),
    // Status
    .rx_done_o(rx_done),
    .tx_done_o(tx_done),
    .crc_err_o(),
    .timeout_o(),
    .response_o(),
    .rx_count_o(),
    .idle_o(),
    // UTMI Interface
    .utmi_data_o     (host_utmi_data_out ),
    .utmi_txvalid_o  (host_utmi_txvalid  ),
    .utmi_txready_i  (host_utmi_txready  ),
    .utmi_data_i     (host_utmi_data_in  ),
    .utmi_rxvalid_i  (host_utmi_rxvalid  ),
    .utmi_rxactive_i (host_utmi_rxactive ),
    .utmi_linestate_i(host_utmi_linestate)
);
wire usb_dxp_io;
wire usb_dxn_io;
wire usb_rxdp_i;
wire usb_rxdn_i;
//==============================================================
//======USB Host PHY
    USB2_0_SoftPHY_Top u_USB_SoftPHY_Top
    (
         .clk_i            (clk_60              )
        ,.rst_i            (reset               )
        ,.fclk_i           (fclk_480M           )
        ,.pll_locked_i     (1'b1                )
        ,.utmi_data_out_i  (host_utmi_data_out  )
        ,.utmi_txvalid_i   (host_utmi_txvalid   )
        ,.utmi_op_mode_i   (2'b00               )
        ,.utmi_xcvrselect_i(2'b00               )
        ,.utmi_termselect_i(1'b0                )
        ,.utmi_data_in_o   (host_utmi_data_in   )
        ,.utmi_txready_o   (host_utmi_txready   )
        ,.utmi_rxvalid_o   (host_utmi_rxvalid   )
        ,.utmi_rxactive_o  (host_utmi_rxactive  )
        ,.utmi_rxerror_o   (host_utmi_rxerror   )
        ,.utmi_linestate_o (host_utmi_linestate )
        ,.usb_dxp_io       (usb_dxp_io          )
        ,.usb_dxn_io       (usb_dxn_io          )
        ,.usb_rxdp_i       (usb_rxdp_i          )
        ,.usb_rxdn_i       (usb_rxdn_i          )
        ,.usb_pullup_en_o  (     )
        ,.usb_term_dp_io   (     )
        ,.usb_term_dn_io   (     )
    );




pulldown(usb_dxp_io);
pulldown(usb_dxn_io);
assign  usb_rxdp_i = usb_dxp_io;
assign  usb_rxdn_i = usb_dxn_io;

Top u_Top(
//interconnection
     .CLK_IN         (clk_in         )
    ,.usb_dxp_io     (usb_dxp_io     )
    ,.usb_dxn_io     (usb_dxn_io     )
    ,.usb_rxdp_i     (usb_rxdp_i     )
    ,.usb_rxdn_i     (usb_rxdn_i     )
    ,.usb_pullup_en_o(               )
    ,.usb_term_dp_io (               )
    ,.usb_term_dn_io (               )
);


endmodule
