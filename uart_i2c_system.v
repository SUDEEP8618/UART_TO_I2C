// =====================================================================
//  uart_i2c_system.v
//  UART Sensor -> FIFO -> I2C Slave, ALL IN ONE MODULE (no sub-files)
//  Target: DE10-Lite (Intel MAX10 10M50DAF484C7G), 50 MHz onboard clock
// =====================================================================
//  UART_RX : receives sensor bytes at 115200, 8-N-1
//  FIFO    : 8 x 8-bit circular buffer (built in-line, no fifo module)
//  I2C SLV : responds to an external I2C MASTER (ESP32/Arduino) that
//            reads bytes out of the FIFO (standard I2C slave-transmit)
//
//  Board wiring (DE10-Lite):
//    clk      -> PIN_P11   (50 MHz onboard clock)
//    rst_n    -> KEY0      (active-low pushbutton)
//    uart_rx  -> any GPIO pin wired to sensor TX
//    scl, sda -> GPIO pins wired to external I2C master, WITH EXTERNAL
//                PULL-UP RESISTORS to 3.3V (required, open-drain bus)
//    led_fifo_count -> LEDR[3:0] (optional debug)
// =====================================================================

module uart_i2c_system #(
    parameter integer CLK_FREQ  = 50_000_000,
    parameter integer BAUD_RATE = 115_200,
    parameter [6:0]   I2C_ADDR  = 7'h50        // 7-bit slave address
)(
    input  wire       clk,
    input  wire       rst_n,

    input  wire       uart_rx,

    inout  wire       scl,
    inout  wire       sda,

    output wire [7:0] led_fifo_count
);

    // -----------------------------------------------------------------
    // 1) Synchronize all asynchronous inputs into the clk domain
    // -----------------------------------------------------------------
    reg [1:0] rx_sync, scl_sync, sda_sync;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_sync  <= 2'b11;
            scl_sync <= 2'b11;
            sda_sync <= 2'b11;
        end else begin
            rx_sync  <= {rx_sync[0],  uart_rx};
            scl_sync <= {scl_sync[0], scl};
            sda_sync <= {sda_sync[0], sda};
        end
    end
    wire rx_in  = rx_sync[1];
    wire scl_in = scl_sync[1];
    wire sda_in = sda_sync[1];

    // -----------------------------------------------------------------
    // 2) UART RECEIVER (115200, 8-N-1)
    // -----------------------------------------------------------------
    localparam integer BAUD_DIV = CLK_FREQ / BAUD_RATE;   // ~434 @ 50MHz

    localparam [1:0] U_IDLE = 2'd0, U_START = 2'd1, U_DATA = 2'd2, U_STOP = 2'd3;

    reg [1:0]  u_state;
    reg [15:0] u_cnt;
    reg [2:0]  u_bit;
    reg [7:0]  u_shift;
    reg [7:0]  rx_byte;
    reg        rx_valid;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            u_state  <= U_IDLE;
            u_cnt    <= 0;
            u_bit    <= 0;
            u_shift  <= 0;
            rx_byte  <= 0;
            rx_valid <= 1'b0;
        end else begin
            rx_valid <= 1'b0;
            case (u_state)
                U_IDLE: begin
                    u_cnt <= 0;
                    if (!rx_in) u_state <= U_START;
                end
                U_START: begin
                    if (u_cnt == (BAUD_DIV/2)) begin
                        u_cnt <= 0;
                        if (!rx_in) begin
                            u_bit  <= 0;
                            u_state <= U_DATA;
                        end else begin
                            u_state <= U_IDLE;      // false start / glitch
                        end
                    end else u_cnt <= u_cnt + 1'b1;
                end
                U_DATA: begin
                    if (u_cnt == BAUD_DIV-1) begin
                        u_cnt   <= 0;
                        u_shift <= {rx_in, u_shift[7:1]};
                        if (u_bit == 3'd7) u_state <= U_STOP;
                        else               u_bit   <= u_bit + 1'b1;
                    end else u_cnt <= u_cnt + 1'b1;
                end
                U_STOP: begin
                    if (u_cnt == BAUD_DIV-1) begin
                        u_cnt    <= 0;
                        rx_byte  <= u_shift;
                        rx_valid <= 1'b1;           // 1-cycle "byte ready" pulse
                        u_state  <= U_IDLE;
                    end else u_cnt <= u_cnt + 1'b1;
                end
                default: u_state <= U_IDLE;
            endcase
        end
    end

    // -----------------------------------------------------------------
    // 3) 8-BYTE FIFO (in-line circular buffer, no separate module)
    // -----------------------------------------------------------------
    reg [7:0] fifo_mem [0:7];
    reg [2:0] wr_ptr, rd_ptr;
    reg [3:0] fifo_cnt;                 // 0..8

    wire fifo_full  = (fifo_cnt == 4'd8);
    wire fifo_empty = (fifo_cnt == 4'd0);
    wire [7:0] fifo_dout = fifo_mem[rd_ptr];

    reg  fifo_pop;                      // pulsed by I2C logic below

    wire wr_en = rx_valid & ~fifo_full;
    wire rd_en = fifo_pop  & ~fifo_empty;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_ptr   <= 0;
            rd_ptr   <= 0;
            fifo_cnt <= 0;
        end else begin
            if (wr_en) begin
                fifo_mem[wr_ptr] <= rx_byte;
                wr_ptr <= wr_ptr + 1'b1;
            end
            if (rd_en) rd_ptr <= rd_ptr + 1'b1;

            case ({wr_en, rd_en})
                2'b10:   fifo_cnt <= fifo_cnt + 1'b1;
                2'b01:   fifo_cnt <= fifo_cnt - 1'b1;
                default: fifo_cnt <= fifo_cnt;         // 00 or 11 -> no change
            endcase
        end
    end

    assign led_fifo_count = {4'b0000, fifo_cnt};

    // -----------------------------------------------------------------
    // 4) I2C SLAVE (slave-transmit: master reads bytes out of the FIFO)
    //    Supports: START, 7-bit addr + R/W, ACK, multi-byte read with
    //    master ACK/NACK, STOP. No clock stretching (kept simple).
    // -----------------------------------------------------------------
    reg scl_d, sda_d;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            scl_d <= 1'b1;
            sda_d <= 1'b1;
        end else begin
            scl_d <= scl_in;
            sda_d <= sda_in;
        end
    end
    wire scl_rise   =  scl_in && !scl_d;
    wire scl_fall   = !scl_in &&  scl_d;
    wire start_cond =  scl_in &&  scl_d && !sda_in &&  sda_d;   // SDA falls, SCL high
    wire stop_cond  =  scl_in &&  scl_d &&  sda_in && !sda_d;   // SDA rises, SCL high

    localparam [2:0] I_IDLE     = 3'd0,
                      I_ADDR     = 3'd1,
                      I_ADDR_ACK = 3'd2,
                      I_TXBYTE   = 3'd3,
                      I_TXACK    = 3'd4;

    reg [2:0] i_state;
    reg [2:0] i_bit;
    reg [7:0] addr_sh;
    reg [7:0] tx_sh;
    reg       sda_oe;      // 1 = actively pull SDA low, 0 = release (open-drain)
    reg       sda_o;

    assign scl = 1'bz;                         // slave never drives SCL
    assign sda = sda_oe ? 1'b0 : 1'bz;          // open-drain: only ever pull low

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            i_state    <= I_IDLE;
            i_bit      <= 0;
            addr_sh    <= 0;
            tx_sh      <= 0;
            sda_oe     <= 1'b0;
            fifo_pop   <= 1'b0;
        end else begin
            fifo_pop <= 1'b0;   // default: 1-cycle pulse only

            if (stop_cond) begin
                i_state <= I_IDLE;
                sda_oe  <= 1'b0;
            end else if (start_cond) begin
                i_state <= I_ADDR;
                i_bit   <= 0;
                sda_oe  <= 1'b0;
            end else begin
                case (i_state)
                    I_IDLE: sda_oe <= 1'b0;

                    // shift in address(7)+R/W(1), MSB first, sampled on SCL rise
                    I_ADDR: begin
                        if (scl_rise) begin
                            addr_sh <= {addr_sh[6:0], sda_in};
                            if (i_bit == 3'd7) i_state <= I_ADDR_ACK;
                            i_bit <= i_bit + 1'b1;
                        end
                    end

                    // decide ACK/NACK on SCL falling edge after 8th bit
                    I_ADDR_ACK: begin
                        if (scl_fall) begin
                            if (addr_sh[7:1] == I2C_ADDR) begin
                                sda_oe <= 1'b1;                  // drive ACK (low)
                                if (addr_sh[0] && !fifo_empty) begin
                                    tx_sh    <= fifo_dout;       // preload 1st byte
                                    fifo_pop <= 1'b1;            // pop it now
                                end
                                i_state <= I_TXBYTE;
                                i_bit   <= 0;
                            end else begin
                                sda_oe  <= 1'b0;                 // NACK: wrong address
                                i_state <= I_IDLE;
                            end
                        end
                    end

                    // shift tx_sh out on SDA, MSB first, data changes on SCL low
                    I_TXBYTE: begin
                        if (scl_fall) begin
                            sda_oe <= tx_sh[7] ? 1'b0 : 1'b1;    // '1'->release, '0'->pull low
                            tx_sh  <= {tx_sh[6:0], 1'b0};
                            if (i_bit == 3'd7) begin
                                i_state <= I_TXACK;
                                i_bit   <= 0;
                            end else begin
                                i_bit <= i_bit + 1'b1;
                            end
                        end
                    end

                    // after 8th bit: release SDA (real SCL edge) so master can
                    // drive ACK(0)/NACK(1), then sample it on the next edge
                    I_TXACK: begin
                        if (scl_fall) begin
                            if (i_bit == 0) begin
                                sda_oe <= 1'b0;                  // release for master's ACK/NACK
                                i_bit  <= 1;
                            end else begin                       // i_bit==1: evaluate master's bit
                                if (!sda_d && !fifo_empty) begin  // ACK & data left -> next byte
                                    // Send bit-1 of the new byte NOW (same edge), since
                                    // there is no spare pulse before the master clocks it in.
                                    sda_oe   <= fifo_dout[7] ? 1'b0 : 1'b1;
                                    tx_sh    <= {fifo_dout[6:0], 1'b0};
                                    fifo_pop <= 1'b1;
                                    i_state  <= I_TXBYTE;
                                    i_bit    <= 1;               // bit-1 already sent
                                end else begin
                                    i_state <= I_IDLE;            // NACK or FIFO empty
                                end
                            end
                        end
                    end

                    default: i_state <= I_IDLE;
                endcase
            end
        end
    end

endmodule