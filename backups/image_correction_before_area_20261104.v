// Full-view preview: 1920x1080 RAW10 -> 1024x600 RAW10.
// Replaces the vendor top-left 1024x600 crop. Sensor timing is unchanged.
// Select whole Bayer 2x2 cells so even/odd colour positions stay aligned.
// This is cell decimation, not an anti-aliased photographic resampler.
// Pixel 0 occupies bits [9:0], matching raw10_unpacket in this project.
// I_raw_frame_start = input tuser; I_raw_frame_end = input tlast (EACH ROW).
// Fixed-rate pipeline: O_raw_tready is unused, as in the vendor integration.
module image_correction #(
    parameter DATA_WIDTH = 40
)(
    input I_clk,
    input I_rst_n,
    input [DATA_WIDTH-1:0] I_raw_data,
    input I_raw_valid,
    input I_raw_frame_start,
    input I_raw_frame_end,
    output reg [DATA_WIDTH-1:0] O_raw_tdata,
    output reg O_raw_tlast,
    output reg O_raw_tvalid,
    output reg O_raw_tuser,
    input O_raw_tready
);
    reg active;
    reg [10:0] source_row;
    reg [3:0] x_phase, y_phase;
    reg [19:0] pending_pair;
    reg pending_valid;
    reg [7:0] output_word;
    reg first_output;

    // Two source pixels form one horizontal Bayer pair. Select 8/15
    // pairs: 960 source pairs -> 512 output pairs -> 256 four-pixel words.
    // Select 5/9 vertical pairs: 540 source pairs -> 300 -> 600 rows.
    wire [10:0] row_now = I_raw_frame_start ? 11'd0 : source_row;
    wire [3:0] yp_now = I_raw_frame_start ? 4'd0 : y_phase;
    wire keep_row = (yp_now >= 4'd4) && (row_now < 11'd1080);
    wire [4:0] y_sum = {1'b0, yp_now} + 5'd5;

    reg [3:0] next_x;
    reg [4:0] phase_sum;
    reg [19:0] next_pending;
    reg next_pending_valid;
    reg emit_word;
    reg [39:0] packed_word;
    integer k;
    always @* begin
        next_x = I_raw_frame_start ? 4'd0 : x_phase;
        next_pending = pending_pair;
        next_pending_valid = I_raw_frame_start ? 1'b0 : pending_valid;
        emit_word = 1'b0;
        packed_word = 40'd0;
        phase_sum = 5'd0;
        for (k = 0; k < 2; k = k + 1) begin
            phase_sum = {1'b0, next_x} + 5'd8;
            if (phase_sum >= 5'd15) begin
                next_x = phase_sum - 5'd15;
                if (next_pending_valid) begin
                    packed_word = {I_raw_data[k*20 +: 20], next_pending};
                    emit_word = 1'b1;
                    next_pending_valid = 1'b0;
                end else begin
                    next_pending = I_raw_data[k*20 +: 20];
                    next_pending_valid = 1'b1;
                end
            end else begin
                next_x = phase_sum[3:0];
            end
        end
    end

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            active <= 1'b0;
            source_row <= 0;
            x_phase <= 0;
            y_phase <= 0;
            pending_pair <= 0;
            pending_valid <= 0;
            output_word <= 0;
            first_output <= 1'b1;
            O_raw_tdata <= 0;
            O_raw_tlast <= 0;
            O_raw_tvalid <= 0;
            O_raw_tuser <= 0;
        end else begin
            O_raw_tvalid <= 1'b0;
            O_raw_tlast <= 1'b0;
            O_raw_tuser <= 1'b0;
            if (I_raw_frame_start) begin
                active <= 1'b1;
                source_row <= 0;
                x_phase <= 0;
                y_phase <= 0;
                pending_valid <= 0;
                output_word <= 0;
                first_output <= 1'b1;
            end
            if (I_raw_valid && (active || I_raw_frame_start)) begin
                x_phase <= next_x;
                if (keep_row) begin
                    pending_pair <= next_pending;
                    pending_valid <= next_pending_valid;
                    if (emit_word) begin
                        O_raw_tdata <= packed_word;
                        O_raw_tvalid <= 1'b1;
                        O_raw_tlast <= (output_word == 8'd255);
                        O_raw_tuser <= first_output;
                        first_output <= 1'b0;
                        output_word <= output_word + 1'b1;
                    end
                end
                if (I_raw_frame_end) begin
                    x_phase <= 0;
                    pending_valid <= 0;
                    output_word <= 0;
                    source_row <= row_now + 1'b1;
                    if (row_now[0])
                        y_phase <= (y_sum >= 5'd9) ? y_sum - 5'd9 : y_sum;
                    if (row_now == 11'd1079)
                        active <= 1'b0;
                end
            end
        end
    end
endmodule
