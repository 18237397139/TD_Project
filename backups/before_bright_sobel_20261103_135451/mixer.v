module mixer #(parameter DEBOUNCE_CYCLES=1000000)(
 input wire I_clk,I_rst_n,I_key_n,
 input wire I_video_vsync,I_video_hsync,I_video_de,
 output wire O_vdieo_rd_en,input wire [23:0] I_video_rd_data,
 output reg O_mixer_vsync,O_mixer_hsync,O_mixer_de,
 output reg [23:0] O_mixer_data);
 reg [1:0] vs,hs,de;
 reg key_meta,key_sync,key_stable;
 reg [31:0] debounce;
 reg [1:0] requested_mode,active_mode;
 reg [10:0] x,y;
 reg de_prev,vs_prev;
 wire [9:0] lum_sum={2'b0,I_video_rd_data[23:16]}+{1'b0,I_video_rd_data[15:8],1'b0}+{2'b0,I_video_rd_data[7:0]};
 wire [7:0] gray=lum_sum[9:2];
 // One synchronous line buffer: |Y-left| + |Y-above|, threshold 48.
 reg [7:0] line_gray [0:1023];
 reg [7:0] above_gray;
 reg [7:0] left_gray,gray_pipe,left_pipe;
 reg [23:0] rgb_pipe;
 reg [10:0] px,py;
 reg vs_pipe,hs_pipe,de_pipe;
 wire [7:0] dx=gray_pipe>=left_pipe ? gray_pipe-left_pipe : left_pipe-gray_pipe;
 wire [7:0] dy=gray_pipe>=above_gray ? gray_pipe-above_gray : above_gray-gray_pipe;
 wire [8:0] gradient={1'b0,dx}+{1'b0,dy};
 // No RAM reset: first row is suppressed; each column is filled before use.
 always @(posedge I_clk) begin
  if(de[1] && x<1024) begin
   above_gray<=line_gray[x[9:0]];
   line_gray[x[9:0]]<=gray;
  end
 end
 always @(posedge I_clk or negedge I_rst_n) begin
  if(!I_rst_n) begin
   left_gray<=0;gray_pipe<=0;left_pipe<=0;rgb_pipe<=0;
   px<=0;py<=0;vs_pipe<=0;hs_pipe<=0;de_pipe<=0;
  end else begin
   vs_pipe<=vs[1];hs_pipe<=hs[1];de_pipe<=de[1];
   px<=x;py<=y;rgb_pipe<=I_video_rd_data;gray_pipe<=gray;
   left_pipe<=left_gray;
   if(de[1]) left_gray<=gray; else left_gray<=0;
  end
 end
 reg [23:0] pixel;
 assign O_vdieo_rd_en=I_video_de;
 always @(posedge I_clk or negedge I_rst_n) begin
  if(!I_rst_n) begin
   key_meta<=1;key_sync<=1;key_stable<=1;debounce<=0;requested_mode<=0;
  end else begin
   key_meta<=I_key_n;key_sync<=key_meta;
   if(key_sync==key_stable) debounce<=0;
   else if(debounce==DEBOUNCE_CYCLES-1) begin
    debounce<=0;key_stable<=key_sync;
    if(!key_sync) requested_mode <= requested_mode==3 ? 0 : requested_mode+1'b1;
   end else debounce<=debounce+1'b1;
  end
 end
 // DDR read latency is two cycles; the line buffer adds one aligned pipeline stage.
 always @(posedge I_clk or negedge I_rst_n) begin
  if(!I_rst_n) begin
   vs<=0;hs<=0;de<=0;O_mixer_vsync<=0;O_mixer_hsync<=0;O_mixer_de<=0;O_mixer_data<=0;
   x<=0;y<=0;de_prev<=0;vs_prev<=0;active_mode<=0;
  end else begin
   vs<={vs[0],I_video_vsync};hs<={hs[0],I_video_hsync};de<={de[0],I_video_de};
   O_mixer_vsync<=vs_pipe;O_mixer_hsync<=hs_pipe;O_mixer_de<=de_pipe;
   O_mixer_data<=de_pipe ? pixel : 24'd0;
   de_prev<=de[1];vs_prev<=vs[1];
   if(vs[1] && !vs_prev) begin x<=0;y<=0;active_mode<=requested_mode;end
   else if(de[1]) x<=x+1'b1;
   else begin x<=0;if(de_prev) y<=y+1'b1;end
  end
 end
 wire [575:0] text_bits=text_row(py-32);
 wire [93:0] red_bits=red_row(py-16),blue_bits=blue_row(py-16);
 always @* begin
  case(active_mode)
   1: pixel={gray_pipe,gray_pipe,gray_pipe};
   2: pixel=gray_pipe>=128 ? 24'hffffff : 24'h000000;
   3: pixel=(px!=0 && py!=0 && gradient>=48) ? 24'hffffff : 24'h000000;
   default: pixel=rgb_pipe;
  endcase
  if(px>=16 && px<110 && py>=16 && py<103) begin
   if(red_bits[px-16]) pixel=24'hff2028;
   if(blue_bits[px-16]) pixel=24'h1c2c50;
  end
  if(px>=128 && px<704 && py>=32 && py<72) begin
   if(text_bits[px-128]) pixel=24'hffffff;
  end
  // Four indicators: original, grayscale, binary, edge. Active mode is green.
  if(py>=80 && py<88 && px>=128 && px<256)
   pixel=((px<160 && active_mode==0)||(px>=160 && px<192 && active_mode==1)||(px>=192 && px<224 && active_mode==2)||(px>=224 && active_mode==3)) ? 24'h20ff60 : 24'h404040;
 end
function [575:0] text_row;
input [6:0] row;
begin case(row)
7'd8: text_row=576'h1f0000001f000007c00100000001000000000000000000000000000070000000000000000000000;
7'd9: text_row=576'h3f0000001f000007c003c03fff03c003f0003fe0001ffe1fff800000f801fe0001fc001f8001fc0;
7'd10: text_row=576'h3fffffff0001f000007c007c03fff07e007f800fff8007ffe1fff800000f801ff8007ff007fe007ff0;
7'd11: text_row=576'h3fffffff1ffffffe007c007c03fff07c007f800fffe00fffe1fff83fffffc01ffc00fff80fff00fff8;
7'd12: text_row=576'h3fffffff1ffffffe007c00f803fff0f8007f800ffff01fffe1fff83fffffc01ffe01fff81fff81fff8;
7'd13: text_row=576'h3c00000f1ffffffe007c00f803c0f1f800ffc00e03f81f83e000f83fffffe0107f03f0383f0f83f038;
7'd14: text_row=576'h3c3c0f0f00000000007c01f003c0f87000ffc00800fc3f03e000f83ffffff0001f03e0083e07c3e008;
7'd15: text_row=576'h3dffffef00f807c0007c00f03fc0f82000fbc000007e3e03e000f8000f01f8000f83e0003e07c3e000;
7'd16: text_row=576'h1ffffe0007c0f80007c00203fc0fc0001f3e000003e3e03e000f8000f00f8000f83e0007e07c3e000;
7'd17: text_row=576'h3c0f003fffffff007c00003fc07e0001f3e000003e3e03e000f8000f00fc000783e0007c03e3e000;
7'd18: text_row=576'hffffff83fffffff7ffffc003f807c0001f1e000001f3e03e000f8000f007e07c7c3f0007c03e3f000;
7'd19: text_row=576'hffffff83fffffff7ffffc7f0000387f83e1f000001f3f03e000f81ffffffc1ff7c1f0007c03e1f000;
7'd20: text_row=576'h3c0f00000000007ffffc7f0000107f83e1f01ff81f1f83e0fff81ffffffc1fffc1f8007c03e1f800;
7'd21: text_row=576'h3c0f00000000007ffffc7f0ffffc7f87e0f81ff81f1fffe0fff81fffffe83fffc0fc007c03e0fc00;
7'd22: text_row=576'h3c0f0007fffff8007c007f0ffffc7f87c0f81ff81f0fffe0fff81fffffe07e1fc07e007c03e07e00;
7'd23: text_row=576'h3fffffff07fffff8007c00780ffffc7c07c0f81ff81f03ffe0fff8000f03e07e0fc03f807c03e03f80;
7'd24: text_row=576'h3fffffff07fffff8007c00780ffffc7c0ffffc1f001f00ffe000f8000f03e07c07c01fc07c03e01fc0;
7'd25: text_row=576'h3fffffff07c00078007c00780f80707c0ffffc1f003f0003e000f8000f03e07c07c00fe07c03e00fe0;
7'd26: text_row=576'h1f001e007c00078007c007807c0787c0ffffc1f003e0003e000f8000f03e07c07c003f03e07e003f0;
7'd27: text_row=576'h7fffff807c00078007c007807c0fa7c1ffffe1f007e0003e000f8000f03e07c078001f83e07c001f8;
7'd28: text_row=576'h1ffffffe07fffff8007c047803e1f37c1f003e1f00fc0003e000f87ffffffe7e0f8000f83e07c000f8;
7'd29: text_row=576'h3fffffff07fffff8007c0e7801f3f3fc1f003e1f83fc0003e000f87ffffffe3f1f8000fc1f0fc000fc;
7'd30: text_row=576'h1de001ee07fffff8007c0f7801ffe3fc3f003f1ffff80003e000f87ffffffe3fff03fffc1fff83fffc;
7'd31: text_row=576'h19e1e1e61c1f1f00007c0ff800ffc3fc3f001f1ffff00003e000f87ffffffe1ffe03fffc0fff03fffc;
7'd32: text_row=576'h1e1e1e07c1f0f80007c0ff8007f83fc3e001f0fffc00003e000f8000f00000ffc03fffc07fe03fffc;
7'd33: text_row=576'h1e1f9e03c1f0fc0007c07f800ffe1fc7e001f81ff000003e000f8000f000003f003fffc01f803fffc;
7'd34: text_row=576'h1fff803c1f07f0007c01fc07fffcfc0000000000000000000000000f000000000000000000000000;
7'd35: text_row=576'h3ffffff3ffe07ff007c00fc3ffffffc0000000000000000000000000f000000000000000000000000;
7'd36: text_row=576'h1fff3ffe1ffe01ff007c00783fe1ff380000000000000000000000000f000000000000000000000000;
7'd37: text_row=576'hfe00ffe0ffc007e007c00101f807e100000000000000000000000000f000000000000000000000000;
7'd38: text_row=576'h600007c0000000c007c00000c000e000000000000000000000000000f000000000000000000000000;
default: text_row=0;
endcase end endfunction
function [93:0] red_row;
input [6:0] row;
begin case(row)
7'd22: red_row=94'h7ffffffffffc00000000000;
7'd23: red_row=94'h7ffffffffffc00000000000;
7'd24: red_row=94'h3ffffffffff800000000000;
7'd27: red_row=94'h1ffffffffff000000000000;
7'd28: red_row=94'hfffffffffe000000000000;
7'd29: red_row=94'hfffffffffe000000000000;
7'd33: red_row=94'h3ffffffff8000000000000;
7'd34: red_row=94'h1ffffffff0000000000000;
7'd35: red_row=94'h1ffffffff0000000000000;
7'd38: red_row=94'h7ffffffc0000000000000;
7'd39: red_row=94'h7ffffffc0000000000000;
7'd40: red_row=94'h3ffffff80000000000000;
7'd44: red_row=94'hfffffe00000000000000;
7'd45: red_row=94'hfffffe00000000000000;
7'd49: red_row=94'h3ffff800000000000000;
7'd50: red_row=94'h1ffff000000000000000;
7'd51: red_row=94'h1ffff000000000000000;
7'd55: red_row=94'h7ffc000000000000000;
7'd56: red_row=94'h3ff8000000000000000;
7'd60: red_row=94'hfe0000000000000000;
7'd61: red_row=94'hfe0000000000000000;
7'd62: red_row=94'h7c0000000000000000;
7'd65: red_row=94'h380000000000000000;
7'd66: red_row=94'h100000000000000000;
7'd67: red_row=94'h100000000000000000;
default: red_row=0;
endcase end endfunction
function [93:0] blue_row;
input [6:0] row;
begin case(row)
7'd23: blue_row=94'h80000000000;
7'd24: blue_row=94'hc0000000000;
7'd25: blue_row=94'h1c0000000000;
7'd26: blue_row=94'h1e0000000000;
7'd27: blue_row=94'h3e0000000000;
7'd28: blue_row=94'h3f0000000000;
7'd29: blue_row=94'h7f0000000000;
7'd30: blue_row=94'h7f8000000000;
7'd31: blue_row=94'hff8000000000;
7'd32: blue_row=94'hffc000000000;
7'd33: blue_row=94'h1ffc000000000;
7'd34: blue_row=94'h1ffe000000000;
7'd35: blue_row=94'h3fff000000000;
7'd36: blue_row=94'h3fff000000000;
7'd37: blue_row=94'h7fff800000000;
7'd38: blue_row=94'h7fff800000000;
7'd39: blue_row=94'hffffc00000000;
7'd40: blue_row=94'hffffc00000000;
7'd41: blue_row=94'h1ffffe00000000;
7'd42: blue_row=94'h1ffffe00000000;
7'd43: blue_row=94'h3ffbff00000000;
7'd44: blue_row=94'h3ff3ff00000000;
7'd45: blue_row=94'h7ff1ff80000000;
7'd46: blue_row=94'h7fe1ff80000000;
7'd47: blue_row=94'hffc0ffc0000000;
7'd48: blue_row=94'hffc0ffc0000000;
7'd49: blue_row=94'h1ff807fe0000000;
7'd50: blue_row=94'h3ff807fe0000000;
7'd51: blue_row=94'h3ff003ff0000000;
7'd52: blue_row=94'h7ff003ff0000000;
7'd53: blue_row=94'h7fe001ff8000000;
7'd54: blue_row=94'hffe001ff8000000;
7'd55: blue_row=94'hffc000ffc000000;
7'd56: blue_row=94'h1ffc000ffc000000;
7'd57: blue_row=94'h1ff80007fe000000;
7'd58: blue_row=94'h3ff80007fe000000;
7'd59: blue_row=94'h3ff00007ff000000;
7'd60: blue_row=94'h7fe03fffff000000;
7'd61: blue_row=94'h7fe03fffff800000;
7'd62: blue_row=94'hffc07fffff800000;
7'd63: blue_row=94'hffc07fffffc00000;
7'd64: blue_row=94'h1ff80ffffffc00000;
7'd65: blue_row=94'h1ff81ffffffe00000;
7'd66: blue_row=94'h3ff01ffffffe00000;
7'd67: blue_row=94'h3ff03fffffff00000;
default: blue_row=0;
endcase end endfunction
endmodule
