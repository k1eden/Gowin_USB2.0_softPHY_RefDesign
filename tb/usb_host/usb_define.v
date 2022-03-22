
//-----------------------------------------------------------------
// Defines:
//-----------------------------------------------------------------

//-----------------------------------------------------------------
// Defines:
//-----------------------------------------------------------------
`define USB_CTRL          16'h0
    `define USB_CTRL_TX_FLUSH                    8
    `define USB_CTRL_TX_FLUSH_SHIFT              8
    `define USB_CTRL_TX_FLUSH_MASK               16'h1

    `define USB_CTRL_PHY_DMPULLDOWN              7
    `define USB_CTRL_PHY_DMPULLDOWN_SHIFT        7
    `define USB_CTRL_PHY_DMPULLDOWN_MASK         16'h1

    `define USB_CTRL_PHY_DPPULLDOWN              6
    `define USB_CTRL_PHY_DPPULLDOWN_SHIFT        6
    `define USB_CTRL_PHY_DPPULLDOWN_MASK         16'h1

    `define USB_CTRL_PHY_TERMSELECT              5
    `define USB_CTRL_PHY_TERMSELECT_SHIFT        5
    `define USB_CTRL_PHY_TERMSELECT_MASK         16'h1

    `define USB_CTRL_PHY_XCVRSELECT_SHIFT        3
    `define USB_CTRL_PHY_XCVRSELECT_MASK         16'h3

    `define USB_CTRL_PHY_OPMODE_SHIFT            1
    `define USB_CTRL_PHY_OPMODE_MASK             16'h3

    `define USB_CTRL_ENABLE_SOF                  0
    `define USB_CTRL_ENABLE_SOF_SHIFT            0
    `define USB_CTRL_ENABLE_SOF_MASK             16'h1

`define USB_STATUS        16'h4
    `define USB_STATUS_SOF_TIME_SHIFT            16
    `define USB_STATUS_SOF_TIME_MASK             16'hffff

    `define USB_STATUS_RX_ERROR                  2
    `define USB_STATUS_RX_ERROR_SHIFT            2
    `define USB_STATUS_RX_ERROR_MASK             16'h1

    `define USB_STATUS_LINESTATE_BITS_SHIFT      0
    `define USB_STATUS_LINESTATE_BITS_MASK       16'h3

`define USB_IRQ_ACK       16'h8
    `define USB_IRQ_ACK_DEVICE_DETECT            3
    `define USB_IRQ_ACK_DEVICE_DETECT_SHIFT      3
    `define USB_IRQ_ACK_DEVICE_DETECT_MASK       16'h1

    `define USB_IRQ_ACK_ERR                      2
    `define USB_IRQ_ACK_ERR_SHIFT                2
    `define USB_IRQ_ACK_ERR_MASK                 16'h1

    `define USB_IRQ_ACK_DONE                     1
    `define USB_IRQ_ACK_DONE_SHIFT               1
    `define USB_IRQ_ACK_DONE_MASK                16'h1

    `define USB_IRQ_ACK_SOF                      0
    `define USB_IRQ_ACK_SOF_SHIFT                0
    `define USB_IRQ_ACK_SOF_MASK                 16'h1

`define USB_IRQ_STS       16'hc
    `define USB_IRQ_STS_DEVICE_DETECT            3
    `define USB_IRQ_STS_DEVICE_DETECT_SHIFT      3
    `define USB_IRQ_STS_DEVICE_DETECT_MASK       16'h1

    `define USB_IRQ_STS_ERR                      2
    `define USB_IRQ_STS_ERR_SHIFT                2
    `define USB_IRQ_STS_ERR_MASK                 16'h1

    `define USB_IRQ_STS_DONE                     1
    `define USB_IRQ_STS_DONE_SHIFT               1
    `define USB_IRQ_STS_DONE_MASK                16'h1

    `define USB_IRQ_STS_SOF                      0
    `define USB_IRQ_STS_SOF_SHIFT                0
    `define USB_IRQ_STS_SOF_MASK                 16'h1

`define USB_IRQ_MASK      16'h10
    `define USB_IRQ_MASK_DEVICE_DETECT           3
    `define USB_IRQ_MASK_DEVICE_DETECT_SHIFT     3
    `define USB_IRQ_MASK_DEVICE_DETECT_MASK      16'h1

    `define USB_IRQ_MASK_ERR                     2
    `define USB_IRQ_MASK_ERR_SHIFT               2
    `define USB_IRQ_MASK_ERR_MASK                16'h1

    `define USB_IRQ_MASK_DONE                    1
    `define USB_IRQ_MASK_DONE_SHIFT              1
    `define USB_IRQ_MASK_DONE_MASK               16'h1

    `define USB_IRQ_MASK_SOF                     0
    `define USB_IRQ_MASK_SOF_SHIFT               0
    `define USB_IRQ_MASK_SOF_MASK                16'h1

`define USB_XFER_DATA     16'h14
    `define USB_XFER_DATA_TX_LEN_SHIFT           0
    `define USB_XFER_DATA_TX_LEN_MASK            16'hffff

`define USB_XFER_TOKEN    16'h18
    `define USB_XFER_TOKEN_START                 31
    `define USB_XFER_TOKEN_START_SHIFT           31
    `define USB_XFER_TOKEN_START_MASK            16'h1

    `define USB_XFER_TOKEN_IN                    30
    `define USB_XFER_TOKEN_IN_SHIFT              30
    `define USB_XFER_TOKEN_IN_MASK               16'h1

    `define USB_XFER_TOKEN_ACK                   29
    `define USB_XFER_TOKEN_ACK_SHIFT             29
    `define USB_XFER_TOKEN_ACK_MASK              16'h1

    `define USB_XFER_TOKEN_PID_DATAX             28
    `define USB_XFER_TOKEN_PID_DATAX_SHIFT       28
    `define USB_XFER_TOKEN_PID_DATAX_MASK        16'h1

    `define USB_XFER_TOKEN_PID_BITS_SHIFT        16
    `define USB_XFER_TOKEN_PID_BITS_MASK         16'hff

    `define USB_XFER_TOKEN_DEV_ADDR_SHIFT        9
    `define USB_XFER_TOKEN_DEV_ADDR_MASK         16'h7f

    `define USB_XFER_TOKEN_EP_ADDR_SHIFT         5
    `define USB_XFER_TOKEN_EP_ADDR_MASK          16'hf

`define USB_RX_STAT       16'h1c
    `define USB_RX_STAT_START_PEND               31
    `define USB_RX_STAT_START_PEND_SHIFT         31
    `define USB_RX_STAT_START_PEND_MASK          16'h1

    `define USB_RX_STAT_CRC_ERR                  30
    `define USB_RX_STAT_CRC_ERR_SHIFT            30
    `define USB_RX_STAT_CRC_ERR_MASK             16'h1

    `define USB_RX_STAT_RESP_TIMEOUT             29
    `define USB_RX_STAT_RESP_TIMEOUT_SHIFT       29
    `define USB_RX_STAT_RESP_TIMEOUT_MASK        16'h1

    `define USB_RX_STAT_IDLE                     28
    `define USB_RX_STAT_IDLE_SHIFT               28
    `define USB_RX_STAT_IDLE_MASK                16'h1

    `define USB_RX_STAT_RESP_BITS_SHIFT          16
    `define USB_RX_STAT_RESP_BITS_MASK           16'hff

    `define USB_RX_STAT_COUNT_BITS_SHIFT         0
    `define USB_RX_STAT_COUNT_BITS_MASK          16'hffff

`define USB_WR_DATA       16'h20
    `define USB_WR_DATA_DATA_SHIFT               0
    `define USB_WR_DATA_DATA_MASK                16'hff

`define USB_RD_DATA       16'h20
    `define USB_RD_DATA_DATA_SHIFT               0
    `define USB_RD_DATA_DATA_MASK                16'hff
// Response values
`define USB_RES_OK           31'h0000
`define USB_RES_NAK          31'hFFFF
`define USB_RES_STALL        31'hFFFE
`define USB_RES_TIMEOUT      31'hFFFD

// USB PID generation macro
`define PID_GENERATE(pid3, pid2, pid1, pid0) ((pid0 << 0) | (pid1 << 1) | (pid2 << 2) | (pid3 << 3) | ((!pid0) << 4) | ((!pid1) << 5) | ((!pid2) << 6)  | ((!pid3) << 7))

// USB PID values
`define PID_OUT        `PID_GENERATE(0,0,0,1) // 16'hE1
`define PID_IN         `PID_GENERATE(1,0,0,1) // 16'h69
`define PID_SOF        `PID_GENERATE(0,1,0,1) // 16'hA5
`define PID_SETUP      `PID_GENERATE(1,1,0,1) // 16'h2D

`define PID_DATA0      `PID_GENERATE(0,0,1,1) // 16'hC3
`define PID_DATA1      `PID_GENERATE(1,0,1,1) // 16'h4B

`define PID_ACK        `PID_GENERATE(0,0,1,0) // 16'hD2
`define PID_NAK        `PID_GENERATE(1,0,1,0) // 16'h5A
`define PID_STALL      `PID_GENERATE(1,1,1,0) // 16'h1E

// Standard requests (via SETUP packets)
`define REQ_GET_STATUS        16'h00
`define REQ_CLEAR_FEATURE     16'h01
`define REQ_SET_FEATURE       16'h03
`define REQ_SET_ADDRESS       16'h05
`define REQ_GET_DESCRIPTOR    16'h06
`define REQ_SET_DESCRIPTOR    16'h07
`define REQ_GET_CONFIGURATION 16'h08
`define REQ_SET_CONFIGURATION 16'h09
`define REQ_GET_INTERFACE     16'h0A
`define REQ_SET_INTERFACE     16'h0B
`define REQ_SYNC_FRAME        16'h0C

// Descriptor types
`define DESC_DEVICE           16'h01
`define DESC_CONFIGURATION    16'h02
`define DESC_STRING           16'h03
`define DESC_INTERFACE        16'h04
`define DESC_ENDPOINT         16'h05
`define DESC_DEV_QUALIFIER    16'h06
`define DESC_OTHER_SPEED_CONF 16'h07
`define DESC_IF_POWER         16'h08

// Device class
`define DEV_CLASS_RESERVED      16'h00
`define DEV_CLASS_AUDIO         16'h01
`define DEV_CLASS_COMMS         16'h02
`define DEV_CLASS_HID           16'h03
`define DEV_CLASS_MONITOR       16'h04
`define DEV_CLASS_PHY_IF        16'h05
`define DEV_CLASS_POWER         16'h06
`define DEV_CLASS_PRINTER       16'h07
`define DEV_CLASS_STORAGE       16'h08
`define DEV_CLASS_HUB           16'h09
`define DEV_CLASS_TMC           16'hFE
`define DEV_CLASS_VENDOR_CUSTOM 16'hFF

// Device Requests (bmRequestType)
`define REQDIR_HOSTTODEVICE        (0 << 7)
`define REQDIR_DEVICETOHOST        (1 << 7)
`define REQTYPE_STANDARD           (0 << 5)
`define REQTYPE_CLASS              (1 << 5)
`define REQTYPE_VENDOR             (2 << 5)
`define REQREC_DEVICE              (0 << 0)
`define REQREC_INTERFACE           (1 << 0)
`define REQREC_ENDPOINT            (2 << 0)
`define REQREC_OTHER               (3 << 0)

// Endpoints
`define ENDPOINT_DIR_MASK          (1 << 7)
`define ENDPOINT_DIR_IN            (1 << 7)
`define ENDPOINT_DIR_OUT           (0 << 7)
`define ENDPOINT_ADDR_MASK         (16'h7F)
`define ENDPOINT_TYPE_MASK         (16'h3)
`define ENDPOINT_TYPE_CONTROL      (0)
`define ENDPOINT_TYPE_ISO          (1)
`define ENDPOINT_TYPE_BULK         (2)
`define ENDPOINT_TYPE_INTERRUPT    (3)
