
module Top(
    input      CLK_IN,
    output     LED,
    inout      usb_dxp_io     ,
    inout      usb_dxn_io     ,
    input      usb_rxdp_i     ,
    input      usb_rxdn_i     ,
    output     usb_pullup_en_o,
    inout      usb_term_dp_io ,
    inout      usb_term_dn_io,
    input cdc_reset

);



wire cdc_reset;
wire [1:0]  PHY_XCVRSELECT      ;
wire        PHY_TERMSELECT      ;
wire [1:0]  PHY_OPMODE          ;
wire [1:0]  PHY_LINESTATE       ;
wire        PHY_TXVALID         ;
wire        PHY_TXREADY         ;
wire        PHY_RXVALID         ;
wire        PHY_RXACTIVE        ;
wire        PHY_RXERROR         ;
wire [7:0]  PHY_DATAIN          ;
wire [7:0]  PHY_DATAOUT         ;
wire        PHY_CLKOUT          ;
wire [9:0]  DESCROM_RADDR       ;
wire [7:0]  DESCROM_RDAT        ;
wire [9:0]  DESC_DEV_ADDR       ;
wire [7:0]  DESC_DEV_LEN        ;
wire [9:0]  DESC_QUAL_ADDR      ;
wire [7:0]  DESC_QUAL_LEN       ;
wire [9:0]  DESC_FSCFG_ADDR     ;
wire [7:0]  DESC_FSCFG_LEN      ;
wire [9:0]  DESC_HSCFG_ADDR     ;
wire [7:0]  DESC_HSCFG_LEN      ;
wire [9:0]  DESC_OSCFG_ADDR     ;
wire [9:0]  DESC_STRLANG_ADDR   ;
wire [9:0]  DESC_STRVENDOR_ADDR ;
wire [7:0]  DESC_STRVENDOR_LEN  ;
wire [9:0]  DESC_STRPRODUCT_ADDR;
wire [7:0]  DESC_STRPRODUCT_LEN ;
wire [9:0]  DESC_STRSERIAL_ADDR ;
wire [7:0]  DESC_STRSERIAL_LEN  ;
wire        DESCROM_HAVE_STRINGS;
wire        RESET_IN;
reg  [7:0]  rst_cnt;
wire [7:0]  usb_txdat;
reg  [11:0] usb_txdat_len;
reg         usb_txcork;
wire        usb_txpop;
wire        usb_txact;
wire [7:0]  usb_rxdat;
wire        usb_rxval;
wire        usb_rxact;
reg         usb_rxrdy;
wire [3:0]  endpt_sel;
wire        rx_fifo_wren;
wire        rx_fifo_empty;
reg         rx_fifo_rden;
wire [7:0]  rx_fifo_data;
wire        rx_fifo_dval;
reg         rx_fifo_dval_d0;
wire        rx_fifo_dval_rise;
wire [9:0]  tx_fifo_wnum;
wire        tx_fifo_empty;
wire [7:0]  tx_fifo_rdat;
wire        tx_fifo_rd;
wire        setup_active;
wire        setup_val;
wire [7:0]  setup_data;
reg         endpt0_send;
reg  [7:0]  endpt0_dat;
reg [11:0]  txdat_len;
wire        pll_locked;

assign RESET_IN = ~pll_locked;
//==============================================================
//======PLL 
Gowin_rPLL u_pll(
    .clkout (fclk_480M ), //output clkout
    .clkoutd(PHY_CLKOUT), //output clkout
    .lock   (pll_locked),
    .clkin  (CLK_IN    )  //input clkin
);

//==============================================================
//======
reg [31:0] led_cnt;
assign LED = (led_cnt >= 10000);
always@(posedge PHY_CLKOUT, posedge RESET_IN) begin
    if (RESET_IN) begin
        led_cnt <= 31'd0;
    end
    else if (led_cnt >= 16'd60000000) begin
        led_cnt <= 31'd0;
    end
    else begin
        led_cnt <= led_cnt + 1'd1;
    end
end

//==============================================================
//======USB Data Loop
wire [7:0] tx_fifo_wr_data;
wire       tx_fifo_wr;
wire [7:0] rx_fifo_wr_data;
wire [9:0] rx_fifo_wnum;

assign tx_fifo_wr_data = rx_fifo_data;
assign tx_fifo_wr      = rx_fifo_dval;
    fifo_sc_top tx_fifo (
        .Clk  (PHY_CLKOUT      ), //input Clk
        .Reset(RESET_IN        ), //input Reset
        .Data (tx_fifo_wr_data ), //input [7:0] Data
        .WrEn (tx_fifo_wr      ), //input WrEn
        .RdEn (tx_fifo_rd      ), //input RdEn
        .Wnum (tx_fifo_wnum    ), //output [6:0] Wnum
        .Q    (tx_fifo_rdat    ), //output [7:0] Q
        .Empty(tx_fifo_empty   ), //output Empty
        .Full (tx_fifo_full    ) //output Full
    );

assign rx_fifo_wren = usb_rxval&(endpt_sel==4'd2);
assign rx_fifo_wr_data = usb_rxdat;
assign tx_fifo_rd = usb_txpop&&(endpt_sel==4'd2);
    fifo_sc_top rx_fifo (
        .Clk  (PHY_CLKOUT     ), //input Clk
        .Reset(RESET_IN       ), //input Reset
        .Data (rx_fifo_wr_data), //input [7:0] Data
        .WrEn (rx_fifo_wren   ), //input WrEn
        .RdEn (rx_fifo_rden   ), //input RdEn
        .Wnum (rx_fifo_wnum   ), //output [6:0] Wnum
        .Q    (rx_fifo_data   ), //output [7:0] Q
        .Empty(rx_fifo_empty  ), //output Empty
        .Full (               )  //output Full
    );
//==============================================================
//======RX FIFO Read Ctrl
    always@(posedge PHY_CLKOUT, posedge RESET_IN) begin
        if (RESET_IN) begin
            rx_fifo_rden <= 1'b0;
        end
        else begin
            if ((rx_fifo_rden == 1'b0)&&(rx_fifo_empty == 0)&&(tx_fifo_full == 1'b0)) begin
                rx_fifo_rden <= 1'b1;
            end
            else begin
                rx_fifo_rden <= 1'b0;
            end
        end
    end
    assign rx_fifo_dval = rx_fifo_rden;


//==============================================================
//======RX Data Ready
always@(posedge PHY_CLKOUT, posedge RESET_IN) begin
    if (RESET_IN) begin
        usb_rxrdy <= 1'b0;
    end
    else begin
        if (endpt_sel == 4'd1) begin
            usb_rxrdy <= 1'b1;
        end
        else if (endpt_sel == 4'd2) begin
            usb_rxrdy <= 1'b1;
        end
        else begin
            usb_rxrdy <= 1'b1;
        end
    end
end

//==============================================================
//======TX Data Cork
always@(posedge PHY_CLKOUT, posedge RESET_IN) begin
    if (RESET_IN) begin
        usb_txcork <= 1'b1;
    end
    else begin
        if (endpt_sel == 4'd0) begin
            usb_txcork <= ~endpt0_send;
        end
        else if (endpt_sel == 4'd2) begin
            usb_txcork <= (tx_fifo_empty)|(tx_fifo_wnum < 1);
        end
        else begin
            usb_txcork <= 1'b1;
        end
    end
end
//==============================================================
//======TX Data Length
always@(posedge PHY_CLKOUT, posedge RESET_IN) begin
    if (RESET_IN) begin
        txdat_len <= 12'd32;
    end
    else if (usb_txact) begin
        txdat_len <= txdat_len;
    end
    else begin
        if (endpt_sel == 4'd0) begin
            txdat_len <= 12'd7;
        end
        else begin
            if (tx_fifo_wnum <= 10'd64) begin
                txdat_len <= tx_fifo_wnum;
            end
            else begin
                txdat_len <= 12'd64;
            end
        end
    end
end


assign usb_txdat = (endpt_sel==4'd0) ? endpt0_dat : tx_fifo_rdat;

wire [7:0]    inf_alter_i;
wire [7:0]    inf_alter_o;
wire [7:0]    inf_sel_o;
wire          inf_set_o;
    USB_Device_Controller_Top u_usb_device_controller_top (
             .clk_i                 (PHY_CLKOUT          )
            ,.reset_i               (RESET_IN            )
            ,.usbrst_o              (usb_busreset        )
            ,.highspeed_o           (usb_highspeed       )
            ,.suspend_o             (usb_suspend         )
            ,.online_o              (usb_online          )
            ,.txdat_i               (usb_txdat           )
            ,.txval_i               (endpt0_send&(endpt_sel==4'd0))//
            ,.txdat_len_i           (txdat_len           )
            ,.txcork_i              (usb_txcork          )
            ,.txiso_pid_i           (4'b0011             )
            ,.txpop_o               (usb_txpop           )
            ,.txact_o               (usb_txact           )
            ,.txpktfin_o            (usb_txpktfin        )
            ,.rxdat_o               (usb_rxdat           )
            ,.rxval_o               (usb_rxval           )
            ,.rxact_o               (usb_rxact           )
            ,.rxrdy_i               (usb_rxrdy           )
            ,.rxpktval_o            (rxpktval            )
            ,.setup_o               (setup_active        )
            ,.endpt_o               (endpt_sel           )
            ,.sof_o                 (usb_sof             )
            ,.inf_alter_i           (8'd0                )
            ,.inf_alter_o           (inf_alter_o         )
            ,.inf_sel_o             (inf_sel_o           )
            ,.inf_set_o             (inf_set_o           )
            ,.descrom_rdata_i       (DESCROM_RDAT        )
            ,.descrom_raddr_o       (DESCROM_RADDR       )
            ,.desc_dev_addr_i       (DESC_DEV_ADDR       )
            ,.desc_dev_len_i        (DESC_DEV_LEN        )
            ,.desc_qual_addr_i      (DESC_QUAL_ADDR      )
            ,.desc_qual_len_i       (DESC_QUAL_LEN       )
            ,.desc_fscfg_addr_i     (DESC_FSCFG_ADDR     )
            ,.desc_fscfg_len_i      (DESC_FSCFG_LEN      )
            ,.desc_hscfg_addr_i     (DESC_HSCFG_ADDR     )
            ,.desc_hscfg_len_i      (DESC_HSCFG_LEN      )
            ,.desc_oscfg_addr_i     (DESC_OSCFG_ADDR     )
            ,.desc_strlang_addr_i   (DESC_STRLANG_ADDR   )
            ,.desc_strvendor_addr_i (DESC_STRVENDOR_ADDR )
            ,.desc_strvendor_len_i  (DESC_STRVENDOR_LEN  )
            ,.desc_strproduct_addr_i(DESC_STRPRODUCT_ADDR)
            ,.desc_strproduct_len_i (DESC_STRPRODUCT_LEN )
            ,.desc_strserial_addr_i (DESC_STRSERIAL_ADDR )
            ,.desc_strserial_len_i  (DESC_STRSERIAL_LEN  )
            ,.desc_have_strings_i   (DESCROM_HAVE_STRINGS)

            ,.utmi_dataout_o        (PHY_DATAOUT       )
            ,.utmi_txvalid_o        (PHY_TXVALID       )
            ,.utmi_txready_i        (PHY_TXREADY       )
            ,.utmi_datain_i         (PHY_DATAIN        )
            ,.utmi_rxactive_i       (PHY_RXACTIVE      )
            ,.utmi_rxvalid_i        (PHY_RXVALID       )
            ,.utmi_rxerror_i        (PHY_RXERROR       )
            ,.utmi_linestate_i      (PHY_LINESTATE     )
            ,.utmi_opmode_o         (PHY_OPMODE        )
            ,.utmi_xcvrselect_o     (PHY_XCVRSELECT    )
            ,.utmi_termselect_o     (PHY_TERMSELECT    )
            ,.utmi_reset_o          (PHY_RESET         )
         );

//==============================================================
//======USB Device descriptor Demo
usb_desc
#(

         .VENDORID    (16'h33AA)
        ,.PRODUCTID   (16'h0000)
        ,.VERSIONBCD  (16'h0100)
        ,.HSSUPPORT   (1       )
        ,.SELFPOWERED (1       )
)
u_usb_desc (
         .CLK                    (PHY_CLKOUT          )
        ,.RESET                  (RESET_IN            )
        ,.i_pid                  (16'd0               )
        ,.i_vid                  (16'd0               )
        ,.i_descrom_raddr        (DESCROM_RADDR       )
        ,.o_descrom_rdat         (DESCROM_RDAT        )
        ,.o_desc_dev_addr        (DESC_DEV_ADDR       )
        ,.o_desc_dev_len         (DESC_DEV_LEN        )
        ,.o_desc_qual_addr       (DESC_QUAL_ADDR      )
        ,.o_desc_qual_len        (DESC_QUAL_LEN       )
        ,.o_desc_fscfg_addr      (DESC_FSCFG_ADDR     )
        ,.o_desc_fscfg_len       (DESC_FSCFG_LEN      )
        ,.o_desc_hscfg_addr      (DESC_HSCFG_ADDR     )
        ,.o_desc_hscfg_len       (DESC_HSCFG_LEN      )
        ,.o_desc_oscfg_addr      (DESC_OSCFG_ADDR     )
        ,.o_desc_strlang_addr    (DESC_STRLANG_ADDR   )
        ,.o_desc_strvendor_addr  (DESC_STRVENDOR_ADDR )
        ,.o_desc_strvendor_len   (DESC_STRVENDOR_LEN  )
        ,.o_desc_strproduct_addr (DESC_STRPRODUCT_ADDR)
        ,.o_desc_strproduct_len  (DESC_STRPRODUCT_LEN )
        ,.o_desc_strserial_addr  (DESC_STRSERIAL_ADDR )
        ,.o_desc_strserial_len   (DESC_STRSERIAL_LEN  )
        ,.o_descrom_have_strings (DESCROM_HAVE_STRINGS)
);

//==============================================================
//======USB SoftPHY
    USB2_0_SoftPHY_Top u_USB_SoftPHY_Top
    (
         .clk_i            (PHY_CLKOUT     )
        ,.rst_i            (PHY_RESET      )
        ,.fclk_i           (fclk_480M      )
        ,.pll_locked_i     (pll_locked     )
        ,.utmi_data_out_i  (PHY_DATAOUT    )
        ,.utmi_txvalid_i   (PHY_TXVALID    )
        ,.utmi_op_mode_i   (PHY_OPMODE     )
        ,.utmi_xcvrselect_i(PHY_XCVRSELECT )
        ,.utmi_termselect_i(PHY_TERMSELECT )
        ,.utmi_data_in_o   (PHY_DATAIN     )
        ,.utmi_txready_o   (PHY_TXREADY    )
        ,.utmi_rxvalid_o   (PHY_RXVALID    )
        ,.utmi_rxactive_o  (PHY_RXACTIVE   )
        ,.utmi_rxerror_o   (PHY_RXERROR    )
        ,.utmi_linestate_o (PHY_LINESTATE  )
        ,.usb_dxp_io       (usb_dxp_io     )
        ,.usb_dxn_io       (usb_dxn_io     )
        ,.usb_rxdp_i       (usb_rxdp_i     )
        ,.usb_rxdn_i       (usb_rxdn_i     )
        ,.usb_pullup_en_o  (usb_pullup_en_o)
        ,.usb_term_dp_io   (usb_term_dp_io )
        ,.usb_term_dn_io   (usb_term_dn_io )
    );


//==============================================================
//======USB CDC Control Parcket Demo
localparam  SET_LINE_CODING = 8'h20;
localparam  GET_LINE_CODING = 8'h21;
localparam  SET_CONTROL_LINE_STATE = 8'h22;
reg [7:0] stage;
reg [7:0] sub_stage;
reg [7:0] s_req_type;
reg [7:0] s_req_code;
reg [15:0] s_ctl_sig;
reg [15:0] s_set_len;
reg [31:0] s_dte_rate;
reg [7:0] s_char_format;
reg [7:0] s_parity_type;
reg [7:0] s_data_bits;


/*
always @(posedge PHY_CLKOUT,negedge cdc_reset) begin
    if (!cdc_reset) begin
        stage <= 8'd0;
        sub_stage <= 8'd0;
        s_req_type <= 8'd0;
        s_req_code <= 8'd0;
        s_ctl_sig <= 16'd0;
        s_set_len <= 16'd0;
        s_dte_rate <= 32'd115200;
        s_char_format <= 8'd0;
        s_parity_type <= 8'd0;
        s_data_bits <= 8'd8;
        endpt0_send <= 1'd0;
        endpt0_dat  <= 8'd0;
    end
    else begin
        if (setup_active) begin
            if (usb_rxval) begin
                case (stage)
                    8'd0 : begin
                        s_req_type <= usb_rxdat;
                        stage <= stage + 8'd1;
                        sub_stage <= 8'd0;
                        endpt0_send <= 1'd0;
                    end
                    8'd1 : begin
                        s_req_code <= usb_rxdat;
                        stage <= stage + 8'd1;
                    end
                    8'd2 : begin
                        if (s_req_code == SET_CONTROL_LINE_STATE) begin
                            //s_ctl_sig[7:0] <= usb_rxdat; //1st fix
                        end
                        stage <= stage + 8'd1;
                    end
                    8'd3 : begin
                        if (s_req_code == SET_CONTROL_LINE_STATE) begin
                            //s_ctl_sig[15:8] <= usb_rxdat; //1st fix
                        end
                        stage <= stage + 8'd1;
                    end
                    8'd4 : begin
                        stage <= stage + 8'd1;
                    end
                    8'd5 : begin
                        stage <= stage + 8'd1;
                    end
                    8'd6 : begin
                        if (s_req_code == SET_LINE_CODING) begin
                            //s_set_len[7:0] <= usb_rxdat; //2nd fix
                        end
                        else if (s_req_code == GET_LINE_CODING) begin
                            //s_set_len[7:0] <= usb_rxdat; //3d fix
                            endpt0_send <= 1'd1;
                        end
                        stage <= stage + 8'd1;
                    end
                    8'd7 : begin
                        if (s_req_code == SET_LINE_CODING) begin
                            //s_set_len[15:8] <= usb_rxdat; //2nd fix
                        end
                        else if (s_req_code == GET_LINE_CODING) begin
                            //s_set_len[15:8] <= usb_rxdat; //3d fix
                            endpt0_send <= 1'd1;
                            endpt0_dat <= s_dte_rate[7:0];
                        end
                        stage <= stage + 8'd1;
                        sub_stage <= 8'd0;
                    end
                    //8'd8 : ;
                endcase
            end
        end
        else if (s_req_code == SET_LINE_CODING) begin
            stage <= 8'd0;
            if ((usb_rxact)&&(endpt_sel == 4'd0)) begin
                if (usb_rxval) begin
                    sub_stage <= sub_stage + 8'd1;
                    if (sub_stage <= 3) begin
                        //s_dte_rate <= {usb_rxdat,s_dte_rate[31:8]}; //4th fix
                    end
                    else if (sub_stage == 4) begin
                        //s_char_format <= usb_rxdat; //4th fix
                    end
                    else if (sub_stage == 5) begin
                        //s_parity_type <= usb_rxdat; //4th fix
                    end
                    else if (sub_stage == 6) begin
                        //s_data_bits <= usb_rxdat; //4th fix
                    end
                end
            end
        end
        else if (s_req_code == GET_LINE_CODING) begin
            stage <= 8'd0;
            if ((usb_txact)&&(endpt_sel == 4'd0)) begin
                if (endpt0_send == 1'b1) begin
                    if (usb_txpop) begin
                        sub_stage <= sub_stage + 8'd1;
                    end
                    if (s_req_code == GET_LINE_CODING) begin
                        if (usb_txpop) begin
                        if (sub_stage <= 0) begin
                                endpt0_dat <= s_dte_rate[15:8];
                        end
                        else if (sub_stage == 1) begin
                                endpt0_dat <= s_dte_rate[23:16];
                        end
                        else if (sub_stage == 2) begin
                                endpt0_dat <= s_dte_rate[31:24];
                        end
                        else if (sub_stage == 3) begin
                                endpt0_dat <= s_char_format;
                        end
                        else if (sub_stage == 4) begin
                                endpt0_dat <= s_parity_type;
                        end
                        else if (sub_stage == 5) begin
                                endpt0_dat <= s_data_bits;
                        end
                        else if (sub_stage == 6) begin
                                endpt0_send <= 1'b0;
                        end
                        else begin
                            endpt0_send <= 1'b0;
                            end
                        end
                    end
                end
            end
            else begin
                sub_stage <= 8'd0;
            end
        end
        else begin
             stage <= 8'd0;
             sub_stage <= 8'd0;
        end
    end
end
*/





endmodule
