// =====================================================================
//  uart_i2c_system_tb.v
//  Single self-contained testbench for uart_i2c_system.v
//    - Drives 3 UART bytes into the DUT (simulated sensor)
//    - Acts as the external I2C MASTER: START -> address+R,
//      reads bytes back with ACK/ACK/NACK -> STOP
//    - Checks the bytes read over I2C match the bytes sent over UART
// =====================================================================
`timescale 1ns/1ps

module uart_i2c_system_tb;

    // -------------------- clock / reset --------------------
    localparam CLK_PERIOD = 20;                  // 50 MHz
    reg clk = 0;
    reg rst_n = 0;
    always #(CLK_PERIOD/2) clk = ~clk;

    // -------------------- UART side --------------------
    localparam integer BAUD_RATE = 115_200;
    localparam integer BIT_TIME  = 1_000_000_000 / BAUD_RATE; // ns per bit
    reg uart_rx = 1'b1;                          // idle high

    // -------------------- I2C side (open-drain) --------------------
    localparam [6:0] SLAVE_ADDR = 7'h50;
    wire scl, sda;
    reg  scl_drv, sda_drv;      // 0 = drive low, 1 = release (z)
    assign scl = scl_drv ? 1'bz : 1'b0;
    assign sda = sda_drv ? 1'bz : 1'b0;
    pullup(scl);                 // models the external pull-up resistors
    pullup(sda);

    wire [7:0] led_fifo_count;

    // -------------------- DUT --------------------
    uart_i2c_system #(
        .CLK_FREQ (50_000_000),
        .BAUD_RATE(BAUD_RATE),
        .I2C_ADDR (SLAVE_ADDR)
    ) dut (
        .clk           (clk),
        .rst_n         (rst_n),
        .uart_rx       (uart_rx),
        .scl           (scl),
        .sda           (sda),
        .led_fifo_count(led_fifo_count)
    );

    // =================================================================
    // UART task: send one byte, 8-N-1
    // =================================================================
    task uart_send_byte(input [7:0] data);
        integer i;
        begin
            uart_rx = 1'b0;              // start bit
            #(BIT_TIME);
            for (i = 0; i < 8; i = i + 1) begin
                uart_rx = data[i];        // LSB first
                #(BIT_TIME);
            end
            uart_rx = 1'b1;              // stop bit
            #(BIT_TIME);
        end
    endtask

    // =================================================================
    // I2C master tasks (bit-banged, behavioral)
    // =================================================================
    localparam I2C_QTR = 2500;   // ns, ~100 kHz I2C bus

    task i2c_idle; begin scl_drv = 1; sda_drv = 1; end endtask

    task i2c_start;
        begin
            sda_drv = 1; scl_drv = 1; #(I2C_QTR);
            sda_drv = 0;               // SDA falls while SCL high -> START
            #(I2C_QTR);
            scl_drv = 0;               // bring SCL low to begin clocking data
            #(I2C_QTR);
        end
    endtask

    task i2c_stop;
        begin
            sda_drv = 0; scl_drv = 0; #(I2C_QTR);
            scl_drv = 1; #(I2C_QTR);
            sda_drv = 1;               // SDA rises while SCL high -> STOP
            #(I2C_QTR);
        end
    endtask

    // master drives 8 bits out (used for the address+R/W byte)
    task i2c_master_send_byte(input [7:0] b);
        integer i;
        begin
            for (i = 7; i >= 0; i = i - 1) begin
                sda_drv = b[i] ? 1 : 0;    // set data while SCL low
                #(I2C_QTR);
                scl_drv = 1;               // SCL high: slave samples
                #(I2C_QTR);
                #(I2C_QTR);
                scl_drv = 0;               // SCL low: change data next
                #(I2C_QTR);
            end
        end
    endtask

    // master reads the ACK bit driven by the slave
    task i2c_master_read_ack(output ack);
        begin
            sda_drv = 1;                  // release SDA to read it
            #(I2C_QTR);
            scl_drv = 1;
            #(I2C_QTR);
            ack = (sda === 1'b0) ? 1'b1 : 1'b0;
            #(I2C_QTR);
            scl_drv = 0;
            #(I2C_QTR);
        end
    endtask

    // master reads one byte the slave is shifting out
    task i2c_master_read_byte(output [7:0] b);
        integer i;
        begin
            sda_drv = 1;                  // release SDA, slave drives it
            for (i = 7; i >= 0; i = i - 1) begin
                #(I2C_QTR);
                scl_drv = 1;
                #(I2C_QTR);
                b[i] = sda;
                #(I2C_QTR);
                scl_drv = 0;
                #(I2C_QTR);
            end
        end
    endtask

    // master sends ACK (continue reading) or NACK (stop reading)
    task i2c_master_send_ack(input ack); // ack=1 -> ACK(pull low), ack=0 -> NACK(release)
        begin
            sda_drv = ack ? 0 : 1;
            #(I2C_QTR);
            scl_drv = 1;
            #(I2C_QTR);
            #(I2C_QTR);
            scl_drv = 0;
            #(I2C_QTR);
        end
    endtask

    // =================================================================
    // Test sequence
    // =================================================================
    reg [7:0] sent0 = 8'hA5, sent1 = 8'h3C, sent2 = 8'h7E;
    reg [7:0] got0, got1, got2;
    reg       ack;
    integer   errors;

    initial begin
        errors  = 0;
        i2c_idle;
        rst_n = 0;
        #200;
        rst_n = 1;
        #200;

        // ---- Step 1: sensor sends 3 bytes over UART into the FIFO ----
        uart_send_byte(sent0);
        uart_send_byte(sent1);
        uart_send_byte(sent2);
        #(BIT_TIME*2);
        $display("[%0t] FIFO count after 3 UART bytes = %0d", $time, dut.fifo_cnt);

        // ---- Step 2: external I2C master reads them back ----
        i2c_start;
        i2c_master_send_byte({SLAVE_ADDR, 1'b1});   // address + Read
        i2c_master_read_ack(ack);
        if (!ack) begin $display("ERROR: no ACK on address byte"); errors = errors + 1; end

        i2c_master_read_byte(got0);
        i2c_master_send_ack(1'b1);                  // ACK -> want another byte

        i2c_master_read_byte(got1);
        i2c_master_send_ack(1'b1);                  // ACK -> want another byte

        i2c_master_read_byte(got2);
        i2c_master_send_ack(1'b0);                  // NACK -> stop after this byte

        i2c_stop;
        #500;

        // ---- Step 3: check results ----
        if (got0 !== sent0) begin $display("ERROR: byte0 exp=%02h got=%02h", sent0, got0); errors = errors + 1; end
        if (got1 !== sent1) begin $display("ERROR: byte1 exp=%02h got=%02h", sent1, got1); errors = errors + 1; end
        if (got2 !== sent2) begin $display("ERROR: byte2 exp=%02h got=%02h", sent2, got2); errors = errors + 1; end

        $display("[%0t] I2C read bytes: %02h %02h %02h", $time, got0, got1, got2);
        $display("[%0t] FIFO count after I2C read = %0d", $time, dut.fifo_cnt);

        if (errors == 0) $display("TEST PASSED");
        else              $display("TEST FAILED with %0d error(s)", errors);

        #200;
        $finish;
    end

    // safety timeout
    initial begin
        #2_000_000;
        $display("TIMEOUT - simulation did not finish");
        $finish;
    end

endmodule