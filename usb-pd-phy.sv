`default_nettype none

module usb_pd_phy #(
         parameter CLK_FREQ = 48000000,
         parameter AW = 32,
         parameter DW = 8
)    (
      // Wishbone interface.
      input wire            wb_clk_i,
      input wire            wb_reset_i,
      input wire [AW-1:0]   wb_adr_i,
      input wire [DW-1:0]   wb_dat_i,
      output wire [DW-1:0]  wb_dat_o,
      input wire            wb_we_i,
      input wire [DW/8-1:0] wb_sel_i,
      output wire           wb_ack_o,
      input wire            wb_cyc_i,
      input wire            wb_stb_i,

      output wire           irq,

      input wire            clk,
     
      output logic          pin_cc_vpwm,
      input wire            cc_va,
      input wire            cc_vb,
      output logic          pin_cc_dir,
      input logic           pin_cc_a_in,
      output logic          pin_cc_a_out,
      input logic           pin_cc_b_in,
      output logic          pin_cc_b_out,

      output logic [7:0]    debug
    );

    logic       data_clk;
    logic       data_busy;
    logic [7:0] data_addr;
    logic [7:0] data_in;
    wire [7:0]  data_out;
    logic       data_we;
      
      // Control stuff
    logic       cc_int;
    wire        transmit;
    logic [3:0] status;
    logic [7:0] rx_len;
    wire [7:0]  tx_len;
    wire [31:0] crc;

    logic cc_clk;
    
    wb_usb_pd_interface #(
        .AW ( AW ),
        .DW ( DW )
    ) interface_inst (
        // Wishbone interface.
        .wb_clk_i   ( wb_clk_i   ),
        .wb_reset_i ( wb_reset_i ),
        .wb_adr_i   ( wb_adr_i   ),
        .wb_dat_i   ( wb_dat_i   ),
        .wb_dat_o   ( wb_dat_o   ),
        .wb_we_i    ( wb_we_i    ),
        .wb_sel_i   ( wb_sel_i   ),
        .wb_ack_o   ( wb_ack_o   ),
        .wb_cyc_i   ( wb_cyc_i   ),
        .wb_stb_i   ( wb_stb_i   ),

        // Interrupt to CPU
        .irq ( irq ),

        // buffer interface
        .data_clk  ( cc_clk    ),
        .data_busy ( data_busy ),
        .data_addr ( data_addr ),
        .data_in   ( data_in   ),
        .data_out  ( data_out  ),
        .data_we   ( data_we   ),

        // Control stuff
        .cc_int      ( cc_int   ),
        .cc_transmit ( transmit ),
        .cc_status   ( status   ),
        .cc_rx_len   ( rx_len   ),
        .cc_tx_len   ( tx_len   ),
        .cc_crc      ( crc      )
    );
    
    assign pin_cc_a_out = 0;
    assign pin_cc_b_out = 0;
    
    localparam CLKDIV = (CLK_FREQ / 2400000) - 1;

    localparam CLKDIV_WIDTH = $clog2(CLKDIV);
    logic [CLKDIV_WIDTH-1:0] clkdiv_counter;
    initial clkdiv_counter = '0;
    always @(posedge clk) begin
        if (clkdiv_counter) clkdiv_counter <= clkdiv_counter - 1;
        else begin
            clkdiv_counter <= CLKDIV;
        end
    end
    assign cc_clk = clkdiv_counter <= (CLKDIV/2);

    // 3.3v/6 == 0.55v the threshold voltage
    logic [5:0] threshold_pwm;
    always @(posedge clk) begin
        threshold_pwm <= { threshold_pwm[4:0], !|threshold_pwm[4:0] };
        pin_cc_vpwm   <= threshold_pwm[5];
    end

    // for debugging
    assign pin_cc_dir = 0;

    wire serial;
    wire serial_latch;
    wire no_signal;

    cc_receiver #() receiver_inst (
        .cc_clk ( cc_clk ), // 1.2MHz clock
        .cc_in  ( pin_cc_a_in | pin_cc_b_in ),

        .serial_out   ( serial       ),
        .serial_latch ( serial_latch ),
        .no_signal    ( no_signal    )
    );

    logic [1:0] out_kword;
    logic [7:0] out;
    logic       out_latch;
    logic [3:0] nibble;
    logic       nibble_latch;
    logic       SOP;
    logic       EOP;
    cc_deserialize #(
    ) deserializer_inst (
        .cc_clk       ( cc_clk       ),
        .serial       ( serial       ),
        .serial_latch ( serial_latch ),
        .no_signal    ( no_signal    ),
        .out_kword    ( out_kword    ),
        .out          ( out          ),
        .out_latch    ( out_latch    ),
        .nibble       ( nibble       ),
        .nibble_latch ( nibble_latch ),
        .SOP          ( SOP          ),
        .EOP          ( EOP          )
    );

    logic [31:0] crc;
    logic        crc_valid;

    logic [3:0]  crc_debug;
    crc32 #() crc_inst (
        .cc_clk       ( cc_clk       ),
        .nibble       ( nibble       ),
        .nibble_latch ( nibble_latch ),
        .SOP          ( SOP          ),
        .crc          ( crc          ),
        .crc_valid    ( crc_valid    ),
        .debug        ( crc_debug    )
    );

    //crc32 #() crc_inst (
    //    .cc_clk      ( cc_clk       ),
    //    .serial_in   ( serial       ),
    //    .serial_latch( serial_latch ),
    //    .SOP         ( SOP          ),
    //    .CRC         ( crc          ),
    //    .CRC_valid   ( crc_match    )
    //);

    localparam STATE_IDLE           = 0;
    localparam STATE_RX_WAITING     = 1;
    localparam STATE_RX_LATCH       = 2;
    localparam STATE_RX_DELAY       = 3;
    localparam STATE_RX_UPDATE_ADDR = 4;
    localparam STATE_FINISHED       = 5;
    logic [3:0] state;

    assign data_busy = state != STATE_IDLE;

    always @(posedge cc_clk) begin
        data_we   <= 0;

        case (state)
        STATE_IDLE: begin
            data_addr <= 8;
            data_in   <= 0;
            
            if (SOP) state <= STATE_RX_WAITING;
        end

        STATE_RX_WAITING: begin
            data_in <= out;
            
            if (no_signal) begin
                rx_len <= data_addr - 8;
                state  <= STATE_IDLE;
            end
            else if (out_latch) begin
                if (crc_valid) begin
                    cc_int <= 1;
                    state  <= STATE_FINISHED;
                end
                else state  <= STATE_RX_LATCH;
            end
        end

        STATE_RX_LATCH: begin
            data_we <= 1;
            state   <= STATE_RX_DELAY;
        end

        STATE_RX_DELAY: begin
            state <= STATE_RX_UPDATE_ADDR;
        end

        STATE_RX_UPDATE_ADDR: begin
            state     <= STATE_RX_WAITING;
            data_addr <= data_addr + 1;
        end

        STATE_FINISHED: begin
            // wait here until there's no signal
            if (no_signal) state <= STATE_IDLE;
        end

        default: begin
            state <= STATE_IDLE;
        end
        endcase
    end
    
    assign status = state;
    
    
    always_comb begin
        debug[0]   = no_signal;
        debug[1]   = SOP ^ |out_kword;
        debug[2]   = crc_valid;
        debug[3]   = nibble_latch;
        debug[7:4] = crc_debug;
        //debug[4]   = SOP;
        //debug[5]   = out_latch;
        //debug[7:6] = out[1:0];
    end
    
endmodule


/*=========================================================================================================
 * Takes the rx serial and converts it to a stream with some control signals
 * 
 *  */
module cc_deserialize #(
)    (
      input wire         cc_clk,
      input wire         serial,
      input wire         serial_latch,
      input wire         no_signal,
      output logic [1:0] out_kword,
      output logic [7:0] out,
      output logic       out_latch,
      output logic [3:0] nibble,
      output logic       nibble_latch,
      output logic       SOP,
      output logic       EOP
    );

    logic [4:0] input_code;
    always @(posedge cc_clk) if (serial_latch) begin
        input_code <= { serial, input_code[4:1] };
    end
    
    logic [4:0] out_nibble;
    decode4b5b #() decoder_inst (.code(input_code), .out(out_nibble));

    
    logic SOPn, SOPs, SOPd;
    cc_SOP_detector #(
        .CABLE_CHECKS ( 0 )
    ) SOP_det_inst (
      .cc_clk ( cc_clk ),
      .in     ( out_nibble ),
      .serial_latch ( serial_latch ),
      .SOP (SOPn), // SOP  - sink device
      .SOPs(SOPs), // SOP' - cable plug near side
      .SOPd(SOPd)  // SOP" - cable plug far side
    );
    assign SOP = SOPn | SOPs | SOPd;

    logic       SOP_found;
    always @(posedge cc_clk) begin
        if (no_signal) SOP_found <= 0;
        else if (SOP)  SOP_found <= 1;
    end

    logic       out_latch_delay; // this delay is to stabilize the output
    logic [2:0] bit_counter;
    logic [2:0] next_bit_counter;
    always_comb begin
        if (!SOP_found)       next_bit_counter = 0;
        else if (SOP)         next_bit_counter = 2; // SOP is latched a couple clocks later
        else if (bit_counter) next_bit_counter = bit_counter - 1;
        else                  next_bit_counter = 4;
    end
    always @(posedge cc_clk) begin
        out_latch_delay <= '0;
        if (serial_latch) begin
            bit_counter <= next_bit_counter;
            if (next_bit_counter == 4) begin
                out_latch_delay <= '1;
            end
        end
    end

    logic       nibble_num;
    logic [9:0] out_raw;
    always @(posedge cc_clk) begin
        if (SOP) begin
            nibble_num <= '0;
            out_raw    <= '0;
            out_latch  <= '0;
        end
        else if (out_latch_delay) begin
            nibble       <= out_nibble;
            nibble_latch <= '1;
            out_raw      <= { out_nibble, out_raw[9:5] };
            nibble_num   <= !nibble_num;
            if (nibble_num) begin
                out       <= { out_raw[8:5], out_raw[3:0] };
                out_kword <= { out_raw[9],   out_raw[4]   };
                out_latch <= '1;
            end
        end
        else begin
            out_latch    <= '0;
            nibble_latch <= '0;
        end
    end

    assign EOP = '0; // TODO: make this detect EOP
    
endmodule
    

/*=========================================================================================================
 * 
 * This module takes the decoded stream and detects valid SOP, SOP' and SOP" ordered sets
 *  */
module cc_SOP_detector #(
      parameter CABLE_CHECKS = 0
)    (
      input wire       cc_clk,
      input wire [4:0] in,
      input wire       serial_latch,
      output logic     SOP, // SOP  - sink device
      output logic     SOPs, // SOP' - cable plug near side
      output logic     SOPd  // SOP" - cable plug far side
    );
    
    localparam INVALID = 2'b00;
    localparam SYNC1   = 2'b01;
    localparam SYNC2   = 2'b10;
    localparam SYNC3   = 2'b11;

    logic [1:0] symbol_reg[15:0];

    integer     i;
    initial begin
        for (i = 0; i < 16; i=i+1) symbol_reg[i] = '0;
    end
    always @(posedge cc_clk) if (serial_latch) begin
        for (i = 1; i < 16; i=i+1) symbol_reg[i-1] <= symbol_reg[i];
        
        case (in)
        5'b10001: symbol_reg[15] <= SYNC1; // K-code Startsynch 1
        5'b10010: symbol_reg[15] <= SYNC2; // K-code Startsynch 2
        5'b10011: symbol_reg[15] <= SYNC3; // K-code Startsynch 3
        default:  symbol_reg[15] <= INVALID;
        endcase
    end

    always_comb begin
        SOP = ((symbol_reg[0]  == SYNC1) &&
               (symbol_reg[5]  == SYNC1) &&
               (symbol_reg[10] == SYNC1) &&
               (symbol_reg[15] == SYNC2));

        SOPs = (CABLE_CHECKS ? ((symbol_reg[0]  == SYNC1) &&
                                (symbol_reg[5]  == SYNC1) &&
                                (symbol_reg[10] == SYNC1) &&
                                (symbol_reg[15] == SYNC2))   : '0);

        SOPd = (CABLE_CHECKS ? ((symbol_reg[0]  == SYNC1) &&
                                (symbol_reg[5]  == SYNC1) &&
                                (symbol_reg[10] == SYNC1) &&
                                (symbol_reg[15] == SYNC2))   : '0);
    end
    
endmodule


   
/*=========================================================================================================
 * 
 * 
 *  */
module cc_receiver #() (
      input wire   cc_clk, // 1.2MHz clock
      input wire   cc_in,

      output logic serial_out,
      output logic serial_latch,
      output logic no_signal
    );

    logic [9:0] shift_reg;
    initial shift_reg = 0;
    always @(posedge cc_clk) begin
        shift_reg <= { shift_reg[8:0], cc_in };
    end

    logic no_signal_src;
    initial no_signal_src = 1;
    always @(posedge cc_clk) begin
        no_signal_src <= shift_reg == 10'h3FF || shift_reg == 10'h000;
    end
    initial no_signal = 1;
    always @(posedge cc_clk) no_signal <= no_signal_src;
    

    //    =       =                   |
    //  7   6   5   4   3   2   1   0   .   .
    //        |               |               |
    //        ^      ==               ==
    // valid when transition between 4/3 and 3==2 and 1==0
    logic next_valid;
    logic next_one;
    logic valid;
    logic one;
    always_comb begin
        next_valid = ((shift_reg[6] ^  shift_reg[5]) &
                      (shift_reg[7] == shift_reg[6]));
        next_one   = (next_valid & 
                      (shift_reg[4] ^  shift_reg[0]));
    end
    initial begin
        valid = 0;
        one   = 0;
    end
    always @(posedge cc_clk) begin
        valid <= next_valid;
        one   <= next_one;
    end

    logic halfbit;
    logic next_halfbit;
    logic next_serial_out;
    logic next_serial_latch;
    always_comb begin
        next_halfbit      = halfbit;
        next_serial_out   = serial_out;
        next_serial_latch = '0;
        
        if (valid && !halfbit) begin
            next_halfbit      = one;
            next_serial_out   = one;
            next_serial_latch = '1;
        end
        else if (valid && halfbit) begin
            next_halfbit = '0;
        end
    end
    
    initial begin
        halfbit      = '0;
        serial_out   = '0;
        serial_latch = '0;
    end
    always @(posedge cc_clk) begin
        halfbit      <= next_halfbit;
        serial_out   <= next_serial_out;
        serial_latch <= next_serial_latch;
    end

endmodule


//  3   2   1   0   .   .
//    |       |       |


/*=========================================================================================================
 * 
 * 
 *  */
module cc_transmitter #() (
      input wire   cc_clk, // 1.2MHz clock
      output logic cc_out,
      input wire   serial_in,
      output logic serial_latch
    );

    
    logic [3:0]   step;
    initial step = 0;
    always @(posedge cc_clk) begin
        step <= { step[2:0], !(&step[3:1]) };
    end

    logic current_bit;
    always @(posedge cc_clk) begin
        if (step[0]) begin
            current_bit <= serial_in;
            cc_out      <= !cc_out;
        end
        else if (step[2] && current_bit) 
            cc_out <= !cc_out;
        else cc_out <= cc_out;
    end

    assign serial_latch = step[0];
    
endmodule


/*=========================================================================================================
 * 
 * 
 *  */
module decode4b5b #() (
      input wire [4:0]   code,
      output logic [4:0] out
    );

    always_comb begin
        case (code)
        5'b11110: out = 5'b00000;
        5'b01001: out = 5'b00001;
        5'b10100: out = 5'b00010;
        5'b10101: out = 5'b00011;
        5'b01010: out = 5'b00100;
        5'b01011: out = 5'b00101;
        5'b01110: out = 5'b00110;
        5'b01111: out = 5'b00111;
        5'b10010: out = 5'b01000;
        5'b10011: out = 5'b01001;
        5'b10110: out = 5'b01010;
        5'b10111: out = 5'b01011;
        5'b11010: out = 5'b01100;
        5'b11011: out = 5'b01101;
        5'b11100: out = 5'b01110;
        5'b11101: out = 5'b01111;
        5'b11000: out = 5'b10001; // K-code Startsynch 1
        5'b10001: out = 5'b10010; // K-code Startsynch 2
        5'b00110: out = 5'b10011; // K-code Startsynch 3
        5'b00111: out = 5'b10101; // K-code Hard Reset 1
        5'b11001: out = 5'b10110; // K-code Hard Reset 2
        5'b01101: out = 5'b11000; // K-code EOP
        default:  out = 5'b11111; // Error
        endcase
    end
endmodule

/*=========================================================================================================
 * 
 * 
 *  */
module encode4b5b #() (
      input wire [4:0]   in,
      output logic [4:0] code
    );

    always_comb begin
        case (code)
        5'b00000: code = 5'b11110;
        5'b00001: code = 5'b01001;
        5'b00010: code = 5'b10100;
        5'b00011: code = 5'b10101;
        5'b00100: code = 5'b01010;
        5'b00101: code = 5'b01011;
        5'b00110: code = 5'b01110;
        5'b00111: code = 5'b01111;
        5'b01000: code = 5'b10010;
        5'b01001: code = 5'b10011;
        5'b01010: code = 5'b10110;
        5'b01011: code = 5'b10111;
        5'b01100: code = 5'b11010;
        5'b01101: code = 5'b11011;
        5'b01110: code = 5'b11100;
        5'b01111: code = 5'b11101;
        5'b10001: code = 5'b11000; // K-code Startsynch 1
        5'b10010: code = 5'b10001; // K-code Startsynch 2
        5'b10011: code = 5'b00110; // K-code Startsynch 3
        5'b10101: code = 5'b00111; // K-code Hard Reset 1
        5'b10110: code = 5'b11001; // K-code Hard Reset 2
        5'b11000: code = 5'b01101; // K-code EOP
        default:  code = 5'b11111; // Error
        endcase
    end
endmodule



    
/*=========================================================================================================
 * 
 * 
 *  */
module crc32 #() (
      input wire          cc_clk,
      input wire [3:0]    nibble,
      input wire          nibble_latch,
      input wire          SOP, // reset
      output logic [31:0] crc,
      output logic        crc_valid,
      output logic [3:0]  debug
    );

    logic bit_in;

    logic [31:0] crc_reg;
    logic [3:0]  nibble_hash;
    logic [31:0] next_hash;
    logic [31:0] next_crc;

    always_comb begin
        nibble_hash = (crc_reg >> 28) ^ { nibble[0], nibble[1], nibble[2], nibble[3] };

        case (nibble_hash)
        4'h0: next_hash = 32'h00000000;
        4'h1: next_hash = 32'h04C11DB7;
        4'h2: next_hash = 32'h09823B6E;
        4'h3: next_hash = 32'h0D4326D9;
        4'h4: next_hash = 32'h130476DC;
        4'h5: next_hash = 32'h17C56B6B;
        4'h6: next_hash = 32'h1A864DB2;
        4'h7: next_hash = 32'h1E475005;
        4'h8: next_hash = 32'h2608EDB8;
        4'h9: next_hash = 32'h22C9F00F;
        4'hA: next_hash = 32'h2F8AD6D6;
        4'hB: next_hash = 32'h2B4BCB61;
        4'hC: next_hash = 32'h350C9B64;
        4'hD: next_hash = 32'h31CD86D3;
        4'hE: next_hash = 32'h3C8EA00A;
        4'hF: next_hash = 32'h384FBDBD;
        endcase

        next_crc = (crc_reg << 4) ^ next_hash;
    end

    initial crc_reg = 32'hFFFFFFFF;
    always @(posedge cc_clk) begin
        if (SOP)               crc_reg <= 32'hFFFFFFFF;
        else if (nibble_latch) crc_reg <= next_crc;
    end

    assign debug = crc_reg[3:0];
    
    assign crc = crc_reg ^ 32'hFFFFFFFF;

    assign crc_valid = crc_reg == 32'hC704DD7B;

endmodule

    

module wb_usb_pd_interface #(
      parameter AW = 32,
      parameter DW = 8
)    (
      // Wishbone interface.
      input wire            wb_clk_i,
      input wire            wb_reset_i,
      input wire [AW-1:0]   wb_adr_i,
      input wire [DW-1:0]   wb_dat_i,
      output reg [DW-1:0]   wb_dat_o,
      input wire            wb_we_i,
      input wire [DW/8-1:0] wb_sel_i,
      output reg            wb_ack_o,
      input wire            wb_cyc_i,
      input wire            wb_stb_i,

      // interrupt to CPU
      output wire           irq,
      
      // buffer interface
      input wire            data_clk,
      input wire            data_busy,
      input wire [7:0]      data_addr,
      input wire [7:0]      data_in,
      output wire [7:0]     data_out,
      input wire            data_we,
      
      // Control stuff
      input wire            cc_int,
      output wire           cc_transmit,
      input wire [3:0]      cc_status,
      input wire [7:0]      cc_rx_len,
      output wire [7:0]     cc_tx_len,
      input wire [31:0]     cc_crc
    );

    // Wishbone Register Addresses
    localparam REG_STATUS  = 3'h0;
    localparam REG_COMMAND = 3'h1;
    localparam REG_TX_LEN  = 3'h2;
    localparam REG_RX_LEN  = 3'h3;
    localparam REG_CRC_0   = 3'h4;
    localparam REG_CRC_1   = 3'h5;
    localparam REG_CRC_2   = 3'h6;
    localparam REG_CRC_3   = 3'h7;

    
    // Only use the LSB nibble for address decoding.
    logic [2:0] reg_addr;
    logic [7:0] mem_addr;

    logic [7:0] reg_dat_o;
    logic [7:0] mem_dat_o;

    logic       reg_stb_valid;
    logic       reg_ack;

    logic       mem_cyc;
    logic       mem_stb;
    logic       mem_ack;

    logic       mem_active;
    logic       reg_active;
    
    always @(posedge wb_clk_i) begin
        reg_addr <= wb_adr_i[3:0];
        mem_addr <= wb_adr_i[7:0];
        reg_active <= 0;
        mem_active <= 0;
        
        if (wb_adr_i < 8) begin reg_active <= 1; end
        else              begin reg_active <= 1; end
    end
    
    always_comb begin
        reg_stb_valid = reg_active && wb_cyc_i && wb_stb_i && !reg_ack;

        mem_cyc       = !reg_active && wb_cyc_i;
        mem_stb       = !reg_active && wb_stb_i;

        if (reg_active) begin
            wb_ack_o = reg_ack;
            wb_dat_o = reg_dat_o;
        end
        else begin
            wb_ack_o = mem_ack;
            wb_dat_o = mem_dat_o;
        end
    end

    // Locate the STB rising edge.
    always @(posedge wb_clk_i) reg_ack <= reg_stb_valid;

    ///////////////////////////////////////
    // Wishbone Registers.
    ///////////////////////////////////////
    logic [7:0]  reg_rx_len;
    logic [7:0]  reg_tx_len;
    logic        reg_int_en;
    logic        reg_int_status;
    logic        reg_clr_int;
    logic        reg_transmit;
    logic [3:0]  reg_status;

    wire [31:0]  wb_crc;
    
    initial begin
        reg_tx_len = 0;
        reg_int_en = 0;
        reg_transmit = 0;
    end
    always @(posedge wb_clk_i) begin
        // Reset
        if (wb_reset_i) begin
            reg_int_en   <= 0;
            reg_transmit <= 0;
        end
        else begin
            reg_clr_int <= 0;
            
            // Register Write
            if (reg_stb_valid && wb_we_i && wb_sel_i[0]) begin
                case (reg_addr)
                REG_COMMAND: begin
                    reg_clr_int  <= wb_dat_i[0];
                    reg_transmit <= wb_dat_i[1];

                    // enable/disable so normal transmits don't affect it
                    if (wb_dat_i[4]) reg_int_en <= 1;
                    if (wb_dat_i[5]) reg_int_en <= 0;
                end
                REG_TX_LEN: reg_tx_len <= wb_dat_i[7:0];
                endcase
            end

            // Register Read
            else if (reg_stb_valid && ~wb_we_i) begin
                reg_dat_o <= '0;
                case (reg_addr)
                REG_STATUS:  reg_dat_o[5:0] <= { data_busy, reg_int_status, reg_status };
                REG_COMMAND: reg_dat_o[7:0] <= '0;
                REG_TX_LEN:  reg_dat_o[7:0] <= reg_tx_len;
                REG_RX_LEN:  reg_dat_o[7:0] <= reg_rx_len;
                REG_CRC_0:   reg_dat_o[7:0] <= wb_crc[ 7: 0];
                REG_CRC_1:   reg_dat_o[7:0] <= wb_crc[15: 8];
                REG_CRC_2:   reg_dat_o[7:0] <= wb_crc[23:16];
                REG_CRC_3:   reg_dat_o[7:0] <= wb_crc[31:24];
                default:     reg_dat_o <= '0;
                endcase
            end
        end
    end
    
    wb_dp_sram#(
        .AW   ( 8   ),
        .SIZE ( 256 )
    ) buf_inst (// Wishbone interface.
        .wb_clk_i   ( wb_clk_i   ),
        .wb_reset_i ( wb_reset_i ),
        .wb_adr_i   ( mem_addr   ),
        .wb_dat_i   ( wb_dat_i   ),
        .wb_dat_o   ( mem_dat_o  ),
        .wb_we_i    ( wb_we_i    ),
        .wb_sel_i   ( wb_sel_i   ),
        .wb_ack_o   ( mem_ack    ),
        .wb_cyc_i   ( mem_cyc    ),
        .wb_stb_i   ( mem_stb    ),

        // second interface
        .data_clk  ( data_clk  ),
        .data_busy ( data_busy ),
        .data_addr ( data_addr ),
        .data_in   ( data_in   ),
        .data_out  ( data_out  ),
        .data_we   ( data_we   )
    );

    always @(posedge wb_clk_i) begin
        if (reg_clr_int) reg_int_status <= '0;
        else if (cc_int) reg_int_status <= '1;
    end

    assign irq = (reg_int_en & reg_int_status) != 0;

    // if clock crosses are needed, they can go here. Note that the interrupt above would need something as well

    always @(posedge wb_clk_i) begin
        reg_status <= cc_status;
        reg_rx_len <= cc_rx_len;
        wb_crc     <= cc_crc;
    end
    always @(posedge data_clk) begin
        cc_transmit <= reg_transmit;
        cc_tx_len   <= reg_tx_len;
    end
    
    
endmodule
