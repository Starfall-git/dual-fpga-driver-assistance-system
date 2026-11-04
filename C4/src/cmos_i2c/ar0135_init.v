
module ar0135_init #(
    parameter CLK_FREQ = 96000000,
    parameter I2C_FREQ = 100000,
    parameter [6:0] I2C_ADDR = 7'h10,
    parameter STARTUP_MS = 20,
    parameter MAX_RETRIES = 3
) (
    input wire clk, input wire rst_n,
    output reg scl, output wire sda_o, output wire sda_oe,
    input wire sda_i,
    output reg done, output reg error,
    output reg [15:0] model_id,
    output reg [7:0] config_index,
    
    input wire runtime_toggle_i,
    input wire [32:0] runtime_command_i,
    output reg runtime_ack_o, runtime_error_o,
    output reg [15:0] runtime_data_o
);
    localparam QUARTER = (CLK_FREQ + 4*I2C_FREQ-1)/(4*I2C_FREQ);
    localparam MS_CYCLES = CLK_FREQ/1000;
    localparam WAIT=0, LOAD=1, START=2, BYTE=3, ACK=4,
               RESTART=5, STOP=6, FINISH=7, HALT=8;
    reg [3:0] state;
    reg [1:0] phase;
    reg [31:0] divider, wait_cycles;
    reg [3:0] byte_index;
    reg [2:0] bit_index;
    reg [7:0] retries;
    reg drive_low, nack;
    reg [15:0] rx_word;
    wire [31:0] init_lut;
    reg runtime; reg [32:0] runtime_command;
    wire [31:0] lut=runtime ? runtime_command[31:0] : init_lut;
    wire [7:0] lut_size;
    wire reading = runtime ? runtime_command[32] : (config_index == 0);
    wire receiving = reading && byte_index >= 4;
    reg [7:0] tx_byte;
    reg [1:0] sda_sync;
    assign sda_o = 1'b0;
    assign sda_oe = drive_low;
    I2C_AR0135_1280720_Config table_i(config_index, init_lut, lut_size);
    always @(posedge clk or negedge rst_n)
        if (!rst_n) sda_sync <= 2'b11;
        else sda_sync <= {sda_sync[0],sda_i};
    always @* begin
        case (byte_index)
            0: tx_byte = {I2C_ADDR,1'b0};
            1: tx_byte = lut[31:24];
            2: tx_byte = lut[23:16];
            3: tx_byte = reading ? {I2C_ADDR,1'b1} : lut[15:8];
            default: tx_byte = lut[7:0];
        endcase
    end
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            runtime<=0;runtime_command<=0;runtime_ack_o<=0;runtime_error_o<=0;runtime_data_o<=0;
            state<=WAIT; phase<=0; divider<=0;
            wait_cycles<=STARTUP_MS*MS_CYCLES;
            scl<=1; drive_low<=0; done<=0; error<=0;
            model_id<=0; config_index<=0; retries<=0;
            byte_index<=0; bit_index<=7; nack<=0; rx_word<=0;
        end else if (state==HALT && done && runtime_toggle_i!=runtime_ack_o) begin
            runtime<=1;runtime_command<=runtime_command_i;runtime_error_o<=0;
            retries<=0;divider<=0;state<=LOAD;
        end else if (state==WAIT) begin
            divider<=0;
            if (wait_cycles!=0) wait_cycles<=wait_cycles-1'b1;
            else state<=LOAD;
        end else if (state==LOAD) begin
            divider<=0; phase<=0;
            if (runtime) begin byte_index<=0;bit_index<=7;nack<=0;rx_word<=0;state<=START;end
            else if (config_index==lut_size) begin done<=1; state<=HALT; end
            else if (lut[31:16]==0) begin
                /* low word is milliseconds, not a bus transaction. */
                wait_cycles<=lut[15:0]*MS_CYCLES;
                config_index<=config_index+1'b1; state<=WAIT;
            end else begin
                byte_index<=0; bit_index<=7; nack<=0; rx_word<=0; state<=START;
            end
        end else if (divider==QUARTER-1) begin
            divider<=0;
            case (state)
                START: begin
                    case(phase)
                        0: begin scl<=1; drive_low<=0; end
                        1: drive_low<=1;
                        3: begin scl<=0; state<=BYTE; end
                    endcase
                    phase<=phase+1'b1;
                end
                BYTE: begin
                    case(phase)
                        0: begin scl<=0; drive_low<=receiving ? 1'b0 : ~tx_byte[bit_index]; end
                        2: scl<=1;
                        3: begin
                            if(receiving) rx_word<={rx_word[14:0],sda_sync[1]};
                            if(bit_index==0) begin state<=ACK; bit_index<=7; end
                            else bit_index<=bit_index-1'b1;
                        end
                    endcase
                    phase<=phase+1'b1;
                end
                ACK: begin
                    case(phase)
                        0: scl<=0;
                        /* SDA changes one quarter after SCL falls, including RX ACK. */
                        1: drive_low<=receiving && byte_index==4;
                        2: scl<=1;
                        3: begin
                            if (!receiving && sda_sync[1]) begin nack<=1; state<=STOP; end
                            else if ((reading && byte_index==5) || (!reading && byte_index==4))
                                state<=STOP;
                            else begin
                                byte_index<=byte_index+1'b1;
                                state<=(reading && byte_index==2) ? RESTART : BYTE;
                            end
                        end
                    endcase
                    phase<=phase+1'b1;
                end
                RESTART: begin
                    case(phase)
                        0: scl<=0;
                        1: drive_low<=0;
                        2: scl<=1;
                        3: begin drive_low<=1; state<=BYTE; end
                    endcase
                    phase<=phase+1'b1;
                end
                STOP: begin
                    case(phase)
                        0: scl<=0;
                        1: drive_low<=1;
                        2: scl<=1;
                        3: begin drive_low<=0; state<=FINISH; end
                    endcase
                    phase<=phase+1'b1;
                end
                FINISH: begin
                    if(nack) begin
                        if(retries==MAX_RETRIES) begin
                            if(runtime) begin runtime_error_o<=1;runtime_ack_o<=runtime_toggle_i;runtime<=0;end
                            else error<=1;
                            state<=HALT;
                        end
                        else begin retries<=retries+1'b1; wait_cycles<=MS_CYCLES; state<=WAIT; end
                    end else begin
                        if(runtime) begin
                            runtime_data_o<=reading ? rx_word : runtime_command[15:0];
                            runtime_ack_o<=runtime_toggle_i;runtime<=0;state<=HALT;
                        end else begin
                            if(reading) model_id<=rx_word;
                            retries<=0; config_index<=config_index+1'b1; state<=LOAD;
                        end
                    end
                end
                default: begin scl<=1; drive_low<=0; end
            endcase
        end else divider<=divider+1'b1;
    end
endmodule
