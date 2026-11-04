module mixer(
 input I_clk,I_rst_n,I_key_n,I_function_key_n,
 input I_video_vsync,I_video_hsync,I_video_de,
 output O_vdieo_rd_en,input[23:0] I_video_rd_data,
 output O_servo_pwm,output reg O_mixer_vsync,O_mixer_hsync,O_mixer_de,output reg[23:0] O_mixer_data);
 reg[1:0] vs,hs,de;reg[10:0] x,y;reg vp,dp;
 assign O_vdieo_rd_en=I_video_de;
 always @(posedge I_clk or negedge I_rst_n)begin
  if(!I_rst_n)begin vs<=0;hs<=0;de<=0;x<=0;y<=0;vp<=0;dp<=0;end
  else begin
   vs<={vs[0],I_video_vsync};hs<={hs[0],I_video_hsync};de<={de[0],I_video_de};vp<=vs[1];dp<=de[1];
   if(vs[1] && !vp)begin x<=0;y<=0;end
   else if(de[1])x<=x+1'b1;
   else begin x<=0;if(dp)y<=y+1'b1;end
  end
 end
 function[7:0] brighten;input[7:0] c;begin
 if(c<64)brighten={c[6:0],1'b0};else if(c<128)brighten=c+8'd64;else brighten=8'd128+{1'b0,c[7:1]};
 end endfunction
 wire[23:0] rgb={brighten(I_video_rd_data[23:16]),brighten(I_video_rd_data[15:8]),brighten(I_video_rd_data[7:0])};
 wire[9:0] lum={2'b0,rgb[23:16]}+{1'b0,rgb[15:8],1'b0}+{2'b0,rgb[7:0]};
 wire[7:0] gray=lum[9:2];
 wire gv,gh,gd,mv,mh,md,cv,ch,cd,nv,nh,nd,ev,ed,edge_pixel,eh,sobel_edge,hv,hh,hd,hole_foreground;
 wire[10:0] hx,hy;
 wire[10:0] gx,gy,mx,my,cx,cy,nx,ny,ex,ey;
 wire[7:0] gaussian;wire opened,closed,contour,weak_edge,strong_edge;
 stream_algorithms algorithms(.clk(I_clk),.rst_n(I_rst_n),.vs(vs[1]),.hs(hs[1]),.de(de[1]),.x(x),.y(y),.gray(gray),
 .gv(gv),.gh(gh),.gd(gd),.gx(gx),.gy(gy),.gaussian(gaussian),
 .mv(mv),.mh(mh),.md(md),.mx(mx),.my(my),.opened(opened),.closed(closed),
 .cv(cv),.ch(ch),.cd(cd),.cx(cx),.cy(cy),.contour(contour),
 .nv(nv),.nh(nh),.nd(nd),.nx(nx),.ny(ny),.weak_edge(weak_edge),.strong_edge(strong_edge),
 .ev(ev),.ed(ed),.ex(ex),.ey(ey),.measurement_edge(edge_pixel),.eh(eh),.sobel_edge(sobel_edge),
 .hv(hv),.hh(hh),.hd(hd),.hx(hx),.hy(hy),.hole_foreground(hole_foreground));
 wire sort_run,sort_rearm;wire[2:0] sort_status;wire sort_good;wire[19:0] sort_gain_x,sort_gain_y;
 wire[1:0] function_id;wire[3:0] algorithm;wire calibrated;wire[8:0] span;
 wire[127:0] extra10;wire[7:0] feature_valid;wire[87:0] landmarks;
 wire[15:0] width10,height10;wire[10:0] bx,by,bw,bh;wire[1:0] measure_status;
 controls_measure controls(.clk(I_clk),.rst_n(I_rst_n),.key_function_n(I_function_key_n),.key_action_n(I_key_n),.frame_vs(vs[1]),
 .edge_vs(ev),.edge_de(ed),.edge_x(ex),.edge_y(ey),.edge_pixel(edge_pixel),
 .function_id(function_id),.algorithm(algorithm),.calibrated(calibrated),.span(span),
 .width10(width10),.height10(height10),.bx(bx),.by(by),.bw(bw),.bh(bh),.measure_status(measure_status),.extra10(extra10),.feature_valid(feature_valid),.landmarks(landmarks),.sort_run(sort_run),.sort_rearm(sort_rearm),.sort_gain_x(sort_gain_x),.sort_gain_y(sort_gain_y));
 reg pv,ph,pd;reg[10:0] px,py;reg[23:0] pixel,selected_pixel;

 silhouette_sort sorter(.clk(I_clk),.rst_n(I_rst_n),.enable(function_id==2 && sort_run),.calibrated(calibrated),.rearm(sort_rearm),
 .vs(ev),.de(ed),.foreground(edge_pixel),.x(ex),.y(ey),.bx(bx),.by(by),.bw(bw),.bh(bh),
 .measure_status(measure_status),.gain_x(sort_gain_x),.gain_y(sort_gain_y),.width10(width10),.height10(height10),
 .status(sort_status),.last_good(sort_good),.servo_out(O_servo_pwm));
 wire[319:0] sort_state_bits=sort_status==1?sort_wait(py-112):sort_status==2?sort_check(py-112):
 sort_status==3?sort_good_row(py-112):sort_status==4?sort_bad_row(py-112):sort_status==5?sort_return(py-112):
 sort_status==6?sort_remove(py-112):sort_pause(py-112);
 reg[11:0] masks,mask_q;reg ov_q,oh_q,od_q;reg[23:0] background_q;

 reg spv,sph,spd;reg[10:0] spx,spy;reg[23:0] source_pixel;
 wire[159:0] all_dimensions={extra10,height10,width10};
 wire[199:0] all_bcd;
 genvar dim;
 generate for(dim=0;dim<10;dim=dim+1)begin:decimal
  bin_to_bcd converter(I_clk,I_rst_n,all_dimensions[dim*16+:16],all_bcd[dim*20+:20]);
 end endgenerate
 wire[3:0] dimension_row=(py-160)>>5;
 wire[19:0] selected_bcd=all_bcd[dimension_row*20+:20];
 wire dim_valid=dimension_row<2 || feature_valid[dimension_row-2];
 reg[4:0] numchar;wire[4:0] numrow=py[4:0];
 wire[15:0] numbits=digit_row({numchar,numrow});
 always @* begin
  numchar=15;
  if(dimension_row<8)case((px-16)>>4)
   0:numchar=dimension_row<2?10+dimension_row:14+dimension_row;
   1:numchar=12;
   2:numchar=dim_valid?{1'b0,selected_bcd[15:12]}:22;
   3:numchar=dim_valid?{1'b0,selected_bcd[11:8]}:22;
   4:numchar=dim_valid?{1'b0,selected_bcd[7:4]}:22;
   5:numchar=dim_valid?13:22;
   6:numchar=dim_valid?{1'b0,selected_bcd[3:0]}:22;
  endcase
  else case((px-16)>>4)
   0:numchar=23;1:numchar=dimension_row==8?1:2;2:numchar=12;
   3:numchar=dim_valid?{1'b0,selected_bcd[15:12]}:22;
   4:numchar=dim_valid?{1'b0,selected_bcd[11:8]}:22;
   5:numchar=dim_valid?{1'b0,selected_bcd[7:4]}:22;
   6:numchar=dim_valid?13:22;7:numchar=dim_valid?{1'b0,selected_bcd[3:0]}:22;
  endcase
 end
 wire[207:0] dimension_label=dimension_row==0?dimlabel_0(numrow):dimension_row==1?dimlabel_1(numrow):
 dimension_row==2?dimlabel_2(numrow):dimension_row==3?dimlabel_3(numrow):dimension_row==4?dimlabel_4(numrow):
 dimension_row==5?dimlabel_5(numrow):dimension_row==6?dimlabel_6(numrow):dimension_row==7?dimlabel_7(numrow):dimension_row==8?dimlabel_8(numrow):dimlabel_9(numrow);
 wire[575:0] title_bits=text_row(py-32);
 wire[93:0] red_bits=red_row(py-16),blue_bits=blue_row(py-16);
 wire[239:0] mode_bits=algorithm==0?label_0(py-32):algorithm==1?label_1(py-32):algorithm==2?label_2(py-32):
 algorithm==3?label_3(py-32):algorithm==4?label_4(py-32):algorithm==5?label_5(py-32):algorithm==6?label_6(py-32):
 label_7(py-32);
 wire[699:0] feature_bits=function_id==0?vision_row(py-76):function_id==1?measure_row(py-76):sort_title(py-76);
 wire[699:0] state_bits=!calibrated?cal_row(py-112):measure_status==0?waiting_row(py-112):measure_status==1?stable_row(py-112):valid_row(py-112);
 wire[699:0] busy_bits=processing_row(py-112);
 wire[899:0] guide_bits=function_id==0?oldguide_row(py-560):function_id==2?sort_guide(py-560):guide_row(py-560);
 always @* begin
  spv=vs[1];sph=hs[1];spd=de[1];spx=x;spy=y;source_pixel=rgb;
  if(function_id==0)case(algorithm)
   1:source_pixel={gray,gray,gray};
   2:begin spv=gv;sph=gh;spd=gd;spx=gx;spy=gy;source_pixel={gaussian,gaussian,gaussian};end
   3:source_pixel=gray>=128?24'hffffff:24'h0;
   4,5:begin spv=mv;sph=mh;spd=md;spx=mx;spy=my;source_pixel=(algorithm==4?opened:closed)?24'hffffff:24'h0;end
   6:begin spv=ev;sph=eh;spd=ed;spx=ex;spy=ey;source_pixel=sobel_edge?24'hffffff:24'h0;end
   7:begin spv=cv;sph=ch;spd=cd;spx=cx;spy=cy;source_pixel=contour?24'hffffff:24'h0;end
   default:source_pixel=rgb;
  endcase
 end
 always @(posedge I_clk or negedge I_rst_n)begin
  if(!I_rst_n)begin pv<=0;ph<=0;pd<=0;px<=0;py<=0;selected_pixel<=0;end
  else begin pv<=spv;ph<=sph;pd<=spd;px<=spx;py<=spy;selected_pixel<=source_pixel;end
 end
 always @* begin
  masks=0;
  if(px>=16 && px<110 && py>=16 && py<103)begin
   if(red_bits[px-16])masks[0]=1;if(blue_bits[px-16])masks[1]=1;
  end
  if(px>=128 && px<704 && py>=32 && py<72)if(title_bits[px-128])masks[2]=1;
  if(function_id==0 && px>=768 && px<1008 && py>=32 && py<64)if(mode_bits[px-768])masks[3]=1;
  if(px>=128 && px<828 && py>=76 && py<108)if(feature_bits[px-128])masks[4]=1;
  if(function_id!=0)begin
   if(px>=32 && px<732 && py>=112 && py<144)begin
    if(function_id==1 || !calibrated)begin if(state_bits[px-32])masks[5]=1;end
    else if(px<352 && sort_state_bits[px-32])masks[5]=1;
   end
   if(function_id==1 && calibrated && measure_status!=0 && py>=160 && py<480)begin
    if(px>=16 && px<144 && numbits[(px-16)&15])masks[6]=1;
    if(px>=800 && px<1008 && dimension_label[px-800])masks[7]=1;
   end
   if(!calibrated && py>=450 && py<555 && (px==256 || px==256+span))masks[8]=1;
   if(calibrated && measure_status!=0 && ((px>=bx && px<bx+bw && (py==by || py==by+bh-1)) || (py>=by && py<by+bh && (px==bx || px==bx+bw-1))))masks[9]=1;
   if((px==160 || px==959) && py>=64 && py<548)masks[10]=1;
   if((py==64 || py==547) && px>=160 && px<960)masks[10]=1;
  end
  if(px>=32 && px<932 && py>=560 && py<592)if(guide_bits[px-32])masks[11]=1;
 end
 always @(posedge I_clk or negedge I_rst_n)begin
  if(!I_rst_n)begin mask_q<=0;background_q<=0;ov_q<=0;oh_q<=0;od_q<=0;end
  else begin mask_q<=masks;background_q<=selected_pixel;ov_q<=pv;oh_q<=ph;od_q<=pd;end
 end
 always @* begin
  pixel=background_q;
  if(mask_q[0])pixel=24'hff2028;if(mask_q[1])pixel=24'h1c2c50;
  if(mask_q[2])pixel=24'hffffff;if(|mask_q[4:3])pixel=24'h20ff60;
  if(|mask_q[7:5])pixel=24'hffff20;if(mask_q[8])pixel=24'hff2020;
  if(mask_q[9])pixel=24'h20ff60;if(mask_q[10])pixel=24'hffcc00;
  if(mask_q[11])pixel=24'hffff20;
 end
 always @(posedge I_clk or negedge I_rst_n)begin
  if(!I_rst_n)begin O_mixer_vsync<=0;O_mixer_hsync<=0;O_mixer_de<=0;O_mixer_data<=0;end
  else begin O_mixer_vsync<=ov_q;O_mixer_hsync<=oh_q;O_mixer_de<=od_q;O_mixer_data<=od_q?pixel:24'b0;end
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
function [239:0] mode_row_0;
input [6:0] row;
begin case(row)
7'd8: mode_row_0=240'hffffff1fffffe0000000000000000000000000000000000000000000000;
7'd9: mode_row_0=240'hffffff1fffffe0000000000000000000000000000000000000000000000;
7'd10: mode_row_0=240'hffffff1fffffe0000000000000000000000000000000000000000000000;
7'd11: mode_row_0=240'he00f07000f01e0000000000000000000000000000000000000000000000;
7'd12: mode_row_0=240'he00787000781e0000000000000000000000000000000000000000000000;
7'd13: mode_row_0=240'he7ffc70ffff9e0000000000000000000000000000000000000000000000;
7'd14: mode_row_0=240'he7ffe70ffff9e0000000000000000000000000000000000000000000000;
7'd15: mode_row_0=240'he783f70ffff9e0000000000000000000000000000000000000000000000;
7'd16: mode_row_0=240'he3e7ff0f0039e0000000000000000000000000000000000000000000000;
7'd17: mode_row_0=240'he1ff270f0039e0000000000000000000000000000000000000000000000;
7'd18: mode_row_0=240'he0ff070ffff9e0000000000000000000000000000000000000000000000;
7'd19: mode_row_0=240'hffffc70ffff9e0000000000000000000000000000000000000000000000;
7'd20: mode_row_0=240'hefe7ff0f0039e0000000000000000000000000000000000000000000000;
7'd21: mode_row_0=240'hef03f70f0039e0000000000000000000000000000000000000000000000;
7'd22: mode_row_0=240'he07f870ffff9e0000000000000000000000000000000000000000000000;
7'd23: mode_row_0=240'he3ff870ffff9e0000000000000000000000000000000000000000000000;
7'd24: mode_row_0=240'he1c0070ffff9e0000000000000000000000000000000000000000000000;
7'd25: mode_row_0=240'he001e7001e00e0000000000000000000000000000000000000000000000;
7'd26: mode_row_0=240'he0ffe7011e10f0000000000000000000000000000000000000000000000;
7'd27: mode_row_0=240'he7ffe707de78f0000000000000000000000000000000000000000000000;
7'd28: mode_row_0=240'he7ff070fde7ef0000000000000000000000000000000000000000000000;
7'd29: mode_row_0=240'he380071f1e3ff8000000000000000000000000000000000000000000000;
7'd30: mode_row_0=240'hffffff1e1fee78000000000000000000000000000000000000000000000;
7'd31: mode_row_0=240'hffffff041fc47c000000000000000000000000000000000000000000000;
7'd32: mode_row_0=240'hffffff000fc030000000000000000000000000000000000000000000000;
7'd33: mode_row_0=240'he0000700000020000000000000000000000000000000000000000000000;
default: mode_row_0=0;
endcase end endfunction
function [239:0] mode_row_1;
input [6:0] row;
begin case(row)
7'd6: mode_row_1=240'h780000000000000000000000000000000000000000000000000000000;
7'd7: mode_row_1=240'h780000000780000000000000000000000000000000000000000000000;
7'd8: mode_row_1=240'hf80000000780000000000000000000000000000000000000000000000;
7'd9: mode_row_1=240'h1ffffff00000780000000000000000000000000000000000000000000000;
7'd10: mode_row_1=240'h1ffffff3ffffff8000000000000000000000000000000000000000000000;
7'd11: mode_row_1=240'h1ffffff3ffffff8000000000000000000000000000000000000000000000;
7'd12: mode_row_1=240'hf3ffffff8000000000000000000000000000000000000000000000;
7'd13: mode_row_1=240'h1c078f00000780000000000000000000000000000000000000000000000;
7'd14: mode_row_1=240'h1ffffff001e0780000000000000000000000000000000000000000000000;
7'd15: mode_row_1=240'h1ffffff001e0780000000000000000000000000000000000000000000000;
7'd16: mode_row_1=240'h1ffffff041e0f80000000000000000000000000000000000000000000000;
7'd17: mode_row_1=240'h1c078f0e1e1fc0000000000000000000000000000000000000000000000;
7'd18: mode_row_1=240'h1c078f1f1e3fc0000000000000000000000000000000000000000000000;
7'd19: mode_row_1=240'h1fff8f0f9e7fc0000000000000000000000000000000000000000000000;
7'd20: mode_row_1=240'h1fff8f07de7bc0000000000000000000000000000000000000000000000;
7'd21: mode_row_1=240'h1fff8f07eefbc0000000000000000000000000000000000000000000000;
7'd22: mode_row_1=240'hf039f73e0000000000000000000000000000000000000000000000;
7'd23: mode_row_1=240'h7fffef011f21e0000000000000000000000000000000000000000000000;
7'd24: mode_row_1=240'h7fffef003f01e0000000000000000000000000000000000000000000000;
7'd25: mode_row_1=240'h7fffef003f81f0000000000000000000000000000000000000000000000;
7'd26: mode_row_1=240'h3c0787007f80f0000000000000000000000000000000000000000000000;
7'd27: mode_row_1=240'h1e0f0780f3c0f8000000000000000000000000000000000000000000000;
7'd28: mode_row_1=240'hfbe0781f3e0f8000000000000000000000000000000000000000000000;
7'd29: mode_row_1=240'h7fc0783e1f878000000000000000000000000000000000000000000000;
7'd30: mode_row_1=240'h1fff07cfc0fc78000000000000000000000000000000000000000000000;
7'd31: mode_row_1=240'h1fffffbdf803f38000000000000000000000000000000000000000000000;
7'd32: mode_row_1=240'h1ff0ff38f001e10000000000000000000000000000000000000000000000;
7'd33: mode_row_1=240'hf0007104000410000000000000000000000000000000000000000000000;
default: mode_row_1=0;
endcase end endfunction
function [239:0] mode_row_2;
input [6:0] row;
begin case(row)
7'd7: mode_row_2=240'hf060001c0180000000000000000000000000000000000000000000000;
7'd8: mode_row_2=240'hf1e0001c03c0000000000000000000000000000000000000000000000;
7'd9: mode_row_2=240'hf0f01ffffbc0000000000000000000000000000000000000000000000;
7'd10: mode_row_2=240'hf0f01ffffbc07ffffe000000000000000000000000000000000000000;
7'd11: mode_row_2=240'h60f0f81ffff9e07ffffe000000000000000000000000000000000000000;
7'd12: mode_row_2=240'he0f078001c01e07ffffe000000000000000000000000000000000000000;
7'd13: mode_row_2=240'h1f0f07c001c01e0000000000000000000000000000000000000000000000;
7'd14: mode_row_2=240'h1f8f03c0ffff8f0000000000000000000000000000000000000000000000;
7'd15: mode_row_2=240'h7cf03e0ffff8f0000000000000000000000000000000000000000000000;
7'd16: mode_row_2=240'h3ef03e0f0078f8000000000000000000000000000000000000000000000;
7'd17: mode_row_2=240'h1ff03f0f0078f8000000000000000000000000000000000000000000000;
7'd18: mode_row_2=240'hff03f8ffff8fc000000000000000000000000000000000000000000000;
7'd19: mode_row_2=240'h7f03f8ffff8fc000000000000000000000000000000000000000000000;
7'd20: mode_row_2=240'h3f03f8f0078f8000000000000000000000000000000000000000000000;
7'd21: mode_row_2=240'h1f83f8f0078e8000000000000000000000000000000000000000000000;
7'd22: mode_row_2=240'hfe3d8ffff8e8000000000000000000000000000000000000000000000;
7'd23: mode_row_2=240'hffbd8ffff8e0000000000000000000000000000000000000000000000;
7'd24: mode_row_2=240'hfffc8f0078e0000000000000000000000000000000000000000000000;
7'd25: mode_row_2=240'h60f7bc0f0078e0000000000000000000000000000000000000000000000;
7'd26: mode_row_2=240'h3e0f13c0ffff8e0000000000000000000000000000000000000000000000;
7'd27: mode_row_2=240'h1e0f03c0ffff8e0000000000000000000000000000000000000000000000;
7'd28: mode_row_2=240'h1e0f03c0f0078e1ffffff800000000000000000000000000000000000000;
7'd29: mode_row_2=240'h1e0f03c0f0078e1ffffff800000000000000000000000000000000000000;
7'd30: mode_row_2=240'h1f1f03c3ffffee1ffffff800000000000000000000000000000000000000;
7'd31: mode_row_2=240'h1fff03c3ffffee0000000000000000000000000000000000000000000000;
7'd32: mode_row_2=240'hffe03c3ffffee0000000000000000000000000000000000000000000000;
7'd33: mode_row_2=240'h3fc03c000000e0000000000000000000000000000000000000000000000;
default: mode_row_2=0;
endcase end endfunction
function [239:0] mode_row_3;
input [6:0] row;
begin case(row)
7'd7: mode_row_3=240'h3c0c00078040000000000000000000000000000000000000000000000;
7'd8: mode_row_3=240'h7ffc3c000780f0f00000003e00000000000000000000000000000000000;
7'd9: mode_row_3=240'h7ffc3e000781f0f00000003e000003fe000000000000000000000000000;
7'd10: mode_row_3=240'h381e1e000783e0f00000003e000003ff800000000000000000000000000;
7'd11: mode_row_3=240'h381e0e0ffff9c0f00000003e000003ffc00000000000000000000000000;
7'd12: mode_row_3=240'h3ffe6f0ffff8c0f00000003e00000203c00000000000000000000000000;
7'd13: mode_row_3=240'h3fffe78ffff800f00000003e00000001e00000000000000000000000000;
7'd14: mode_row_3=240'h3c00f38f078000f00000003e00000001e00000000000000000000000000;
7'd15: mode_row_3=240'h3c00ff8f078000f01f003f3e01f80001e00000000000000000000000000;
7'd16: mode_row_3=240'h1ffffff8f0783f8f07fc07fbe07fe0003e00000000000000000000000000;
7'd17: mode_row_3=240'h1ffffbf8f0783f8f0ffe0fffe0fff000fc00000000000000000000000000;
7'd18: mode_row_3=240'hf03c0f0783f8f0f1f0fc7e1f0f803fc00000000000000000000000000;
7'd19: mode_row_3=240'h41fc1e0f03c3c0f1e0f1f83e3e0780ff800000000000000000000000000;
7'd20: mode_row_3=240'h1f1df1e0f03c3c0f1e079f01e3c03c3fe000000000000000000000000000;
7'd21: mode_row_3=240'hfbc78f0f03c3c0f1fff9f01e3c03c3f8000000000000000000000000000;
7'd22: mode_row_3=240'h3ff1670f03e3c0f1fff9f01e3c03c7e0000000000000000000000000000;
7'd23: mode_row_3=240'h7ff83f8f01e3c0f1fff9f01e3c03c780000000000000000000000000000;
7'd24: mode_row_3=240'h779c3f8781f3c0f00079f01e3c03c780000000000000000000000000000;
7'd25: mode_row_3=240'h77cf3f87f8fbc0f000f8f83e1e07c780200000000000000000000000000;
7'd26: mode_row_3=240'hefe78307f87bc0f081f0fc7e1f0f87c0e00000000000000000000000000;
7'd27: mode_row_3=240'heff90001f073e0f0fff07ffe0fff03ffe00000000000000000000000000;
7'd28: mode_row_3=240'h1ef7c78000027f0f0ffe03ffe07fe01ffe00000000000000000000000000;
7'd29: mode_row_3=240'h1cf3f7f80000ff8f03f801f3e01f8007f800000000000000000000000000;
7'd30: mode_row_3=240'h1c71fff9fffff78000000000000000000000000000000000000000000000;
7'd31: mode_row_3=240'h87e7879ffffe38000000000000000000000000000000000000000000000;
7'd32: mode_row_3=240'h7e1001ffff818000000000000000000000000000000000000000000000;
7'd33: mode_row_3=240'h1e00000000008000000000000000000000000000000000000000000000;
default: mode_row_3=0;
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
function [239:0] label_0; input[4:0] row; begin case(row)
5'd6: label_0=240'he000000018000060000000;
5'd7: label_0=240'he00000003c0000f0000fc0;
5'd8: label_0=240'he00000003c0000f0003ff0;
5'd9: label_0=240'he000000018000060007ff8;
5'd10: label_0=240'he00000000000000000f87c;
5'd11: label_0=240'he00000000000000000f03c;
5'd12: label_0=240'he07e039c1c77c07e71f03e;
5'd13: label_0=240'he0ff07dc1c7fe07f71e01e;
5'd14: label_0=240'he1ff0ffc1c7ff07ff1e01e;
5'd15: label_0=240'he1c00f3c1c78f870f1e01e;
5'd16: label_0=240'he1fe0e1c1c70787071e01e;
5'd17: label_0=240'he1ff0e1c1c70787071e01e;
5'd18: label_0=240'he1c38e1c1c70787071e03e;
5'd19: label_0=240'he1c38e1c1c70787070f03c;
5'd20: label_0=240'he1e38e1c1c78f87070f87c;
5'd21: label_0=240'he1ff8e1c1c7ff070707ff8;
5'd22: label_0=240'he1df0e1c1c77f070703ff0;
5'd23: label_0=240'he1ce0e1c1c73c070700fc0;
5'd24: label_0=240'h780000000000;
5'd25: label_0=240'h3c1000000000;
5'd26: label_0=240'h3ff000000000;
5'd27: label_0=240'h1ff000000000;
5'd28: label_0=240'h7e000000000;
default: label_0=0; endcase end endfunction
function [239:0] label_1; input[4:0] row; begin case(row)
5'd7: label_1=240'h1fc0;
5'd8: label_1=240'h3ff0;
5'd9: label_1=240'h3ff8;
5'd10: label_1=240'h207c;
5'd11: label_1=240'h3c;
5'd12: label_1=240'he0787e738001e;
5'd13: label_1=240'he070ff7b8000e;
5'd14: label_1=240'h70f1ff7f87f0e;
5'd15: label_1=240'h70f1c00787f0e;
5'd16: label_1=240'h38e1fe0387f0e;
5'd17: label_1=240'h39e1ff038700e;
5'd18: label_1=240'h39c1c3838701e;
5'd19: label_1=240'h1dc1c3838701c;
5'd20: label_1=240'h1f81e3838707c;
5'd21: label_1=240'h1f81ff8387ff8;
5'd22: label_1=240'hf81df0387ff0;
5'd23: label_1=240'hf01ce0381fc0;
5'd24: label_1=240'h700000000000;
5'd25: label_1=240'h780000000000;
5'd26: label_1=240'h780000000000;
5'd27: label_1=240'h3f0000000000;
5'd28: label_1=240'h1f0000000000;
5'd29: label_1=240'hf0000000000;
default: label_1=0; endcase end endfunction
function [239:0] label_2; input[4:0] row; begin case(row)
5'd6: label_2=240'h70000000000000300000000000000000;
5'd7: label_2=240'h703fe000000000780000000000001fc0;
5'd8: label_2=240'h707fe000000000780000000000003ff0;
5'd9: label_2=240'h70ffe000000000300000000000003ff8;
5'd10: label_2=240'h70f0e00000000000000000000000207c;
5'd11: label_2=240'h70e0e00000000000000000000000003c;
5'd12: label_2=240'h1ce1c1c70e0e000e703f038fc1f838383f0001e;
5'd13: label_2=240'h1ee1c1c7070e001f707f838fe1fc38387f8000e;
5'd14: label_2=240'h1fe1c1c703fe003ff0ff838ff1fe3838ff87f0e;
5'd15: label_2=240'h1e1c1c707fe003cf0e00388710e3838e007f0e;
5'd16: label_2=240'he1c1c70ffe003870ff0383f07e3838ff07f0e;
5'd17: label_2=240'he1c1c71f0e003870ff838ff1fe3838ff8700e;
5'd18: label_2=240'he1c1c71e0e003870e1c39fe3fc3838e1c701e;
5'd19: label_2=240'he1c1c71e0e003870e1c39f03e03838e1c701c;
5'd20: label_2=240'he1e3c71f0e003870f1c39c13823c78f1c707c;
5'd21: label_2=240'he1ffc70ffe003870ffc39ff3fe3ff8ffc7ff8;
5'd22: label_2=240'he1df870ffe003870ef838ff1fe3bf0ef87ff0;
5'd23: label_2=240'he1cf0703fe003870e70383e07c39e0e701fc0;
default: label_2=0; endcase end endfunction
function [239:0] label_3; input[4:0] row; begin case(row)
5'd6: label_3=240'h60000;
5'd7: label_3=240'hf07fc;
5'd8: label_3=240'hf0ffc;
5'd9: label_3=240'h61ffc;
5'd10: label_3=240'h1e1c;
5'd11: label_3=240'h1c1c;
5'd12: label_3=240'h703f381f80e7071c1c;
5'd13: label_3=240'h703fb83fc1f7070e1c;
5'd14: label_3=240'h387ff87fc3ff0707fc;
5'd15: label_3=240'h3878787003cf070ffc;
5'd16: label_3=240'h1c70387f8387071ffc;
5'd17: label_3=240'h1cf0387fc387073e1c;
5'd18: label_3=240'h1ce03870e387073c1c;
5'd19: label_3=240'hee03870e387073c1c;
5'd20: label_3=240'hfc03878e387073e1c;
5'd21: label_3=240'hfc0387fe387071ffc;
5'd22: label_3=240'h7c03877c387071ffc;
5'd23: label_3=240'h780387383870707fc;
5'd24: label_3=240'h38000000000000000;
5'd25: label_3=240'h3c000000000000000;
5'd26: label_3=240'h3c000000000000000;
5'd27: label_3=240'h1f800000000000000;
5'd28: label_3=240'hf800000000000000;
5'd29: label_3=240'h7800000000000000;
default: label_3=0; endcase end endfunction
function [239:0] label_4; input[4:0] row; begin case(row)
5'd7: label_4=240'hfc0;
5'd8: label_4=240'h3ff0;
5'd9: label_4=240'h7ff8;
5'd10: label_4=240'hf87c;
5'd11: label_4=240'hf03c;
5'd12: label_4=240'he703e01e71f03e;
5'd13: label_4=240'h1f70ff07f71e01e;
5'd14: label_4=240'h3ff0ff87ff1e01e;
5'd15: label_4=240'h3cf1e3cf0f1e01e;
5'd16: label_4=240'h3871c1ce071e01e;
5'd17: label_4=240'h3871ffce071e01e;
5'd18: label_4=240'h3871ffce071e03e;
5'd19: label_4=240'h387001ce070f03c;
5'd20: label_4=240'h387083cf0f0f87c;
5'd21: label_4=240'h3870ff87ff07ff8;
5'd22: label_4=240'h3870ff03ff03ff0;
5'd23: label_4=240'h38707e01f700fc0;
5'd24: label_4=240'h700000;
5'd25: label_4=240'h700000;
5'd26: label_4=240'h700000;
5'd27: label_4=240'h700000;
5'd28: label_4=240'h700000;
default: label_4=0; endcase end endfunction
function [239:0] label_5; input[4:0] row; begin case(row)
5'd6: label_5=240'he0000;
5'd7: label_5=240'he3f80;
5'd8: label_5=240'he3fe0;
5'd9: label_5=240'he3ff8;
5'd10: label_5=240'he2078;
5'd11: label_5=240'he003c;
5'd12: label_5=240'hf81f80fc0e001c;
5'd13: label_5=240'h3fc1fc1fe0e000e;
5'd14: label_5=240'h3fe1fe3ff0e000e;
5'd15: label_5=240'h78f10e7878e000e;
5'd16: label_5=240'h70707e7038e000e;
5'd17: label_5=240'h7ff1fe7038e000e;
5'd18: label_5=240'h7ff3fc7038e001e;
5'd19: label_5=240'h73e07038e001c;
5'd20: label_5=240'h20f3827878e207c;
5'd21: label_5=240'h3fe3fe3ff0e3ff8;
5'd22: label_5=240'h3fc1fe1fe0e3ff0;
5'd23: label_5=240'h1f807c0fc0e1fc0;
default: label_5=0; endcase end endfunction
function [239:0] label_6; input[4:0] row; begin case(row)
5'd6: label_6=240'h700000000e000000e0000000;
5'd7: label_6=240'h7001ff800e000000e00003f0;
5'd8: label_6=240'h7001ff800e000000e00003fc;
5'd9: label_6=240'h7001ff800e000000e00003fe;
5'd10: label_6=240'h700007800e000000e000021e;
5'd11: label_6=240'h700007800e000000e000000e;
5'd12: label_6=240'h1f03be077c007800e07c03ce07e000e;
5'd13: label_6=240'h7f83ff07fe007800e1fe0fee0ff003e;
5'd14: label_6=240'h7fc3ff87ff0ff800e1ff0ffe1ff80fc;
5'd15: label_6=240'hf1e3c7c7878ff800e3c79e1e3c3c1f0;
5'd16: label_6=240'he0e383c7038ff800e3839c0e381c3c0;
5'd17: label_6=240'hffe383c703807800e3ff9c0e381c780;
5'd18: label_6=240'hffe383c703807800e3ff9c0e381c700;
5'd19: label_6=240'he383c703807800e0039c0e381c702;
5'd20: label_6=240'h41e3c7c787807800e1079e1e3c3c786;
5'd21: label_6=240'h7fc3ff87ff1ff800e1ff0ffe1ff83fe;
5'd22: label_6=240'h7f83bf877f1ff800e1fe07fe0ff03fe;
5'd23: label_6=240'h3f039e073c1ff800e0fc03ce07e00fc;
5'd24: label_6=240'h3c00000000000000000000000000;
5'd25: label_6=240'h1e08000000000000000000000000;
5'd26: label_6=240'h1ff8000000000000000000000000;
5'd27: label_6=240'hff8000000000000000000000000;
5'd28: label_6=240'h3f0000000000000000000000000;
default: label_6=0; endcase end endfunction
function [239:0] label_7; input[4:0] row; begin case(row)
5'd7: label_7=240'hffc000000000000000000003f80;
5'd8: label_7=240'h1000000000040000ffc000000000000800000003fe0;
5'd9: label_7=240'h1c00000000070000ffc000000000000e00000003ff8;
5'd10: label_7=240'h1c0000000007000003c000000000000e00000002078;
5'd11: label_7=240'h1c0000000007000003c000000000000e0000000003c;
5'd12: label_7=240'hff7e01f9ce3ffc3e03c039c38383f07f87381f8001c;
5'd13: label_7=240'hff7f03fdee3fde3c03c03dc38387f87f8fb83fc000e;
5'd14: label_7=240'hff7f87fdfe3fce787fc03fc3838ffc7f9ff87fe000e;
5'd15: label_7=240'h1c43c7001e070f787fc003c3839e1e0e1e78f0f000e;
5'd16: label_7=240'h1c01c7f80e0707f07fc001c3839c0e0e1c38e07000e;
5'd17: label_7=240'h1c01c7fc0e0703e003c001c3839c0e0e1c38e07000e;
5'd18: label_7=240'h1c01c70e0e0703e003c001c3839c0e0e1c38e07001e;
5'd19: label_7=240'h1c01c70e0e0707f003c001c3839c0e0e1c38e07001c;
5'd20: label_7=240'h1c43c78e0e070f7803c001c3c79e1e0e1c38f0f207c;
5'd21: label_7=240'hfc7f87fe0e3f0f38ffc001c3ff8ffc7e1c387fe3ff8;
5'd22: label_7=240'hfc7f077c0e3f1e3cffc001c3bf07f87e1c383fc3ff0;
5'd23: label_7=240'hf83e07380e3e3c1effc001c39e03f07c1c381f81fc0;
default: label_7=0; endcase end endfunction
function [699:0] vision_row; input[4:0] row; begin case(row)
5'd6: vision_row=700'h1c070003870000000;
5'd7: vision_row=700'h7ffefe01c0700ffc307ffffe;
5'd8: vision_row=700'h7ffefe01c0700ffe387ffffe;
5'd9: vision_row=700'h7ffefe01c0380f07387ffffe;
5'd10: vision_row=700'h71ce3801cff80707b8780e1e;
5'd11: vision_row=700'h71ce3801cff87ffffc7bff1e;
5'd12: vision_row=700'h7ffe3809cffc7fffbc7bffde;
5'd13: vision_row=700'h7ffe381fce1c70c3bc7bc3fe;
5'd14: vision_row=700'h7ffe383dce1c70c3be79e77e;
5'd15: vision_row=700'h71cefe39ce1e7fffbe78fe5e;
5'd16: vision_row=700'h71cefe79c71e7fffbe78ff1e;
5'd17: vision_row=700'h7ffefe71c73f107c3f7ffffe;
5'd18: vision_row=700'h7ffe3821c73f38ff3e7f83fe;
5'd19: vision_row=700'h7ffe3801c7be3eeffa787f1e;
5'd20: vision_row=700'h1c03801c3f21ff3fa79ff1e;
5'd21: vision_row=700'h7ffe3801c3f01fdcb879e01e;
5'd22: vision_row=700'h7ffff801c1e01def38781fde;
5'd23: vision_row=700'h7ffef801c1e03bf7f87bffde;
5'd24: vision_row=700'h1c0fc01c7f03bbbb87bfe1e;
5'd25: vision_row=700'h1c0bf003ff87b9eb879801e;
5'd26: vision_row=700'hffff0e7fff3ef38fb87ffffe;
5'd27: vision_row=700'hffff027ffc1f61fff87ffffe;
5'd28: vision_row=700'hffff007fc00e21fbf87ffffe;
5'd29: vision_row=700'h400f0b878001e;
default: vision_row=0; endcase end endfunction
function [699:0] measure_row; input[4:0] row; begin case(row)
5'd6: measure_row=700'h1c000101e00000000000000000e02000e000;
5'd7: measure_row=700'h7fffe1c1ff781e00007fffe00000020e0f000e000;
5'd8: measure_row=700'h7fffe1d9fff81e00007fffe0003f038e27800e1fe;
5'd9: measure_row=700'h7000e1d9fff01e00007fffe000ffc7ce73c00e1fe;
5'd10: measure_row=700'h7fffe1d98761fffff87001e000ffc3fee1c7ffdfe;
5'd11: measure_row=700'h7000e1d98741fffff87001e001f0c0fffff7ffc70;
5'd12: measure_row=700'h7fffe1d9b711fffff87001e001e0003fffe7ffc70;
5'd13: measure_row=700'h7fffe1d9b7381e00007001e001e0020ecfe78e070;
5'd14: measure_row=700'h1d9b7781e00807001e001e0071e00078e070;
5'd15: measure_row=700'h3fffff9d9b7f01e01e07fffe001f007fcffc78f070;
5'd16: measure_row=700'h3fffff9d9b7601e03e07fffe000f807fcffc787070;
5'd17: measure_row=700'h1d9b7201e07c07fffe0007c03f8ffc787070;
5'd18: measure_row=700'h7fffe1d9b7001e0f807071e0003e000ee1c387070;
5'd19: measure_row=700'h70f0e1d9b7201e1f000071e0001f020ee1c387070;
5'd20: measure_row=700'h7fffe1d9b7e01e0e0000f1e00007838effc387370;
5'd21: measure_row=700'h70f0e1d9bfe01e040000e0e00003c7ceffc387bf0;
5'd22: measure_row=700'h7fffe1d838701e000001e0e00003e3fee1c383bfc;
5'd23: measure_row=700'hf001d858701e000003c0f001ffe0fee1c383cff;
5'd24: measure_row=700'hfffff1d8fc701e000007c0f001ffe03effc381c1e;
5'd25: measure_row=700'hfffff1c0ee700e00001f80f001ffe20effc381e06;
5'd26: measure_row=700'hf001c1cf780ffc007f007800000f0ee1c3fcf00;
5'd27: measure_row=700'h3fffffdfb87b80ff801fe007c0000071ee1c3f8f80;
5'd28: measure_row=700'h3fffffcfb873807f800f80030000007fc7dc0f8700;
5'd29: measure_row=700'hf90230000000600010000007fc7dc000200;
5'd30: measure_row=700'h3f8000000000;
default: measure_row=0; endcase end endfunction
function [699:0] sorting_row; input[4:0] row; begin case(row)
5'd6: sorting_row=700'h13800800000801c02000000000e038008100038060000000;
5'd7: sorting_row=700'h3b801e7fff9e01c07000000000f03801e780038cf03ffffc;
5'd8: sorting_row=700'h7b801c7fff9c01c0f00000007fff3803c780039c703ffffc;
5'd9: sorting_row=700'h33803c7fffbc7ffe780000007fff3803c3c0039e703ffffc;
5'd10: sorting_row=700'h7fffb87003b87ffe3c0000007ffffe0781e0038e78003c00;
5'd11: sorting_row=700'h7fff8070e3887ffe1e0000000038fe0f01f07ffe38003c00;
5'd12: sorting_row=700'h7fff8070e38001c00e0000000038fe1f00f87ffe38003c00;
5'd13: sorting_row=700'h7801f77fb9f01c00c00006003fe387e007e7fff3c003c00;
5'd14: sorting_row=700'h7801f77fb9fffff640000f003fe38fc003f03873c003c00;
5'd15: sorting_row=700'h7001f70e39fffffe00000f003fe387ffffe03873e003c00;
5'd16: sorting_row=700'h77f9c77ff9cffff70000060039e382ffff40387be003c00;
5'd17: sorting_row=700'h77f9c77ff9c000078000000038efc0ffff003823f003c00;
5'd18: sorting_row=700'h77f9c70039c1e00380000007ffeff0f038003803e003c00;
5'd19: sorting_row=700'h71e1c77fb9cffff3c0000007ffefe0f0380ffffbe003c00;
5'd20: sorting_row=700'h71e1c77fb9cffff3e0000007ffe3e0f0380ffffba003c00;
5'd21: sorting_row=700'h71e1c771bbcffff3e0000001388380f03c0ffffba003c00;
5'd22: sorting_row=700'h271e9c771bbc1e0c3e0000603bbc380f03c0038038003c00;
5'd23: sorting_row=700'he71edc771bbc1e1e3a0000f03b9e380f03c0038038003c00;
5'd24: sorting_row=700'he77efc77f9dc1e3c380000f0738f380701e0038038003c00;
5'd25: sorting_row=700'h6e7efc77f9ce1e3838000060738f380701f0038038003c00;
5'd26: sorting_row=700'h6e7ffc7819e41e1038000000e3f73e07f8fc0380387ffffe;
5'd27: sorting_row=700'h7e3fbc3f01e01fe03800000023f23e07f87e0380387ffffe;
5'd28: sorting_row=700'h7c039c3f00e00fe03800000001f01e03f83c0380387ffffe;
5'd29: sorting_row=700'h3800081f008007e038000000000000000008038038000000;
default: sorting_row=0; endcase end endfunction
function [699:0] cal_row; input[4:0] row; begin case(row)
5'd6: cal_row=700'h1e01800600003c1c000000380700800e00000000ce060000060000010000000000000000000000000000000000000000001800000038;
5'd7: cal_row=700'h1e03c00700003c3ff000003c0701c00e00000001ce0e00001e0ffff3c000000000000000000000000000000000000000003c007ffe38;
5'd8: cal_row=700'h1e07c1ff9fe03c3fe07000380703f00e00000003ce0f0fff8e0ffff380fc3c0f80f1f800000000000000000001f003f0003c007ffe38;
5'd9: cal_row=700'h1e0f81ff9fe03c07e07c0fffc701f80e000000038e070fff8f0ffff783ff3c0f80f3fe00000000000000000007f80ffc7ffffe7ffe38;
5'd10: cal_row=700'h1e0701ffdfe1bd87007f0fffc700fe0e0000000f0e078fff870e007703ff3c0f81e3ff0000000000000000000ffc0ffc7ffffe0000ff;
5'd11: cal_row=700'h1e0201e1e381ff87007f8fffdfc03f8e0000000ffff38070378e1c7107c31e1f81e20f8000000000000000000f1e1f0c7ffffe0000ff;
5'd12: cal_row=700'h1e0000e0f383bdc7007f8e01dfc00fce0000000ffffbc070738e1c7007801e1fc1e0078000000000000000000f1e1e0078001e0000ff;
5'd13: cal_row=700'h1e03f7fff387bdfff0798e0ddfc0078e000000001ff9c07079ceff73e7801e1fc1e00780001803c3cf03c3cf1e0f1e0078001e7fff38;
5'd14: cal_row=700'h1e03f7fff1c73dfff078001e0700018e000000001c1fe0703fceff73e7800e1fc3c00f80003c07f7ff07f7ff1e0f1e000000007fff3c;
5'd15: cal_row=700'h1e03f7ffcfce3dfff078000e0700000e0000000e1c1fc0703fee1c73e7c00f1cc3c01f00003c0fffff0fffff1e0f1f007ffffe7fff3c;
5'd16: cal_row=700'h1e038718cfe23ce780781fffe70fffffc000000ffe0fc0701fcefff383e00f3ce3c0fe0000180f9f9f0f9f9f1e0f0f807ffffe01c07c;
5'd17: cal_row=700'h3e038718cfe13ce780781fffff0fffffc000000fffce00700e4efff381f00f3ce381fc0000000f0f0f0f0f0f1e0f07c07ffffe01c1fe;
5'd18: cal_row=700'h3e0387ffcdf7bccfc0781fffff8fffffc0000003ffc700700f0e007380f8073c6383f00000000f0f0f0f0f0f1e0f03e000380019ccfe;
5'd19: cal_row=700'h3f0387ffcdf3bc1fc07803871fe00f0e000000071fc78070078eff73807c07b86787c00000000f0f0f0f0f0f1e0f01f00038701ddcbf;
5'd20: cal_row=700'h7f138718cde3fc3fe078038387c00e0e000000079c3fc0703fceff73801e07f87787800000000f0f0f0f0f0f1e0f00783ff87039ce3f;
5'd21: cal_row=700'h771b8718cde1e03f607803c387401e0e00000003fc1fc0703fcee377800f03f87707808000000f0f0f0f0f0f0f1f003c3ff87039ce3a;
5'd22: cal_row=700'h779f8718cdc1f017707801e7c7003c0e00000001f80fc0703fcee377800f83f03f07c18000180f0f0f0f0f0f0f1e003e3ff87879ce3a;
5'd23: cal_row=700'he39f87ffefc0f807607801ff8700780e00001c04f800c070018ee37787ff83f03f03ff80003c0f0f0f0f0f0f07fe1ffe00387871c738;
5'd24: cal_row=700'h1e3cf87ffefc07c07207800fe0701f20e00001c1c7e000070000eff3b87ff83f03f03ff80003c0f0f0f0f0f0f07fc1ffe0038fc71c738;
5'd25: cal_row=700'h3c1e78718efc03f07207801fc0707e3ee00000e0e7ffe00703e0eff39c7ff81f03e00ff0000180f0f0f0f0f0f01f01ffe0039fcf1c7b8;
5'd26: cal_row=700'h7c1f78718e1c01fc7000007ff870fc3ff00000e0effffefffffcf033c80000000000000000000000000000000000000007ffffe21c238;
5'd27: cal_row=700'hf80fbc7d8f1c007e700000f8ff7cf80ff0000070fe3ffcfffffc7e03c00000000000000000000000000000000000000007fffce01f838;
5'd28: cal_row=700'h70071c7d8600001e700000f07e7c6001e00000707e0c0cfffc1c7e01c00000000000000000000000000000000000000007ffe0e01f838;
5'd29: cal_row=700'h2002083d84000004700000400c3c0000600000003800000000003e010000000000000000000000000000000000000000000000400f838;
default: cal_row=0; endcase end endfunction
function [699:0] waiting_row; input[4:0] row; begin case(row)
5'd5: waiting_row=700'h1c00000000000000000000000000000000000000000000000000000000000000000000000000;
5'd6: waiting_row=700'h380000003800030000c0600380c0003c00038060000000000000000000000000003c0000e1c000000038000070000403c038003c00;
5'd7: waiting_row=700'h38007ffe3800078001c0f0019ffe003c00038cf03ffffc0000003ffffc1ffff8003c0000f1fe000000383ffe707fde03e038003c00;
5'd8: waiting_row=700'h38007ffe3800078001c0f07fdffe7ffffe039c703ffffc0000003ffffc1ffff8003c007ff1fe000000383ffe767ffe07e038003c00;
5'd9: waiting_row=700'h38007ffe38000f0001c0e07fc0c07ffffe039e703ffffc0000003ffffc1ffff81ffff87ff87e0000003bfffe767ffc0ff0383ffffc;
5'd10: waiting_row=700'h3ffffc000efe000f00ffeffe7feffe7ffffe038e78003c000000001f00001c00781ffff87ffc700000003bc0707661d80e79fe3ffffc;
5'd11: waiting_row=700'h3ffffc000efe001e00ffeffe38fffe78001e7ffe38003c000000000fc0001c00781ffff8381e700000003bc4387661d01e3dfe3ffffc;
5'd12: waiting_row=700'h3ffffc7ffefe003c00ffeffe39fcce78001e7ffe38003c0000000003e0001c0078003c001c2e700000003bce1e766dc47c1ffe003c00;
5'd13: waiting_row=700'h383c1c7ffe38003c003870381fecce0000007fff3c003c0000000000f8001c0078003c000ff5fe0000603bdffe766dceffff38003c00;
5'd14: waiting_row=700'h381c1c7ffe38003c003870381f8ffe1ffff003873c003c000000000078001c0078003c0007e1fe0000f03bdffe766dde7fff38003c00;
5'd15: waiting_row=700'h381c1c070e3c007e003878380f0ffe1ffff003873e003c000000000038001ffff87ffffe01f1fe0000f03bfffe766dfc2ffa7c003c00;
5'd16: waiting_row=700'h381c1c070efc007e0038fff81f82f01ffff00387be003c00000000fffffe1ffff87ffffe07fe780000603bc800766dd80000fc7ffffe;
5'd17: waiting_row=700'h383e1c3feefe00ff0038fff8ffeffc00000003823f003c00000000fffffe1ffff87ffffe03fef80000003bc1c0766dc818c1fe7ffffe;
5'd18: waiting_row=700'h38ff1c3feefe00ef001cf7f879fedf00000003803e003c00000000fffffe1c1c780000007fe5fc0000003bc1c0766dc038c1be7ffffe;
5'd19: waiting_row=700'h39ff9c070e7f01e7801cc73c6048067ffffeffffbe003c00000000003800001c780038707ff3fc0000003bc1c0766dc839cebf007e00;
5'd20: waiting_row=700'h3be7dc070e3f03c7801dc71c3ffffc7ffffeffffba003c00000000003800003c783ff8707ff9fe0000003bdffe766df839ce3f00ff00;
5'd21: waiting_row=700'h3fc3fc7ffe3a03c3c00fc71c3ffffc7ffffeffffba003c000000000038000038383ff870707ef60000003bdffe766ff81dcc3a01ff80;
5'd22: waiting_row=700'h3b81dc7ffe3a0783e00f871c00380000e380038038003c000000000038000078383ff878381f760000603bdffe760e1c1ddc3a03ffc0;
5'd23: waiting_row=700'h39009c7ffe380f81f007871c1ff87000e3c0038038003c000000e000380000f03c0038783ce6720000f03bc1c076161c1d9c3807bde0;
5'd24: waiting_row=700'h38001c000e381f00f80f871c1ff87030e3c0038038003c000000e0003c0001f03c0038fc1fe0720000f03801c0763f1c0c08380f3cf8;
5'd25: waiting_row=700'h38001c000e383e007c1fe71e00387070e1f0038038003c00000070003c0007e03c003bfc0f8072000060383fc0703b9c0e00383e3c7c;
5'd26: waiting_row=700'h38001c7ffe387c007e3df78e00387070e0f80380387ffffe000070003fe01fc01e7fffde03f070000000381fff7073de7fff387c3c3e;
5'd27: waiting_row=700'h3fc01c7ffe387c003efcfbff7ffffe7fe07e0380387ffffe000038001fe07f801f7fff8e01ff700000003f9ffe7ee1ee7fff38783c1e;
5'd28: waiting_row=700'h3fc01c7ffe3838001c707be67ffffe7fe03c0380387ffffe000038000fe03e000c7fff0e007e700000003f807e3ee1ce7fff38203c04;
5'd29: waiting_row=700'h1f801c0000381000082011e40000003fc00c038038000000000000000000180004000004000e700000001f00003e408c000038003c00;
default: waiting_row=0; endcase end endfunction
function [699:0] stable_row; input[4:0] row; begin case(row)
5'd6: stable_row=700'h700001c07003806000000001c03800003001e0080000000000000018000071c0003c000000000000007000043c0000003c00;
5'd7: stable_row=700'h700000e070038cf03ffffc01c0383fff7001e01e000000000000003c000039fe003c000000001ffff8707fde3c07fe003c00;
5'd8: stable_row=700'h70001ff7fe039c703ffffc01c0383fff707fff9c00000000000e003c001ff9fe003c007000001ffff8767ffe3c07fe7ffffe;
5'd9: stable_row=700'h70001ff7fe039e703ffffc3ffe383fff787fffbc00000000001c7ffffe1ffc7e3ffffc3800001c0038767ffc3c07fe7ffffe;
5'd10: stable_row=700'h70000e3870038e78003c003ffefe38077801e0380000000000387ffffe1ffe703ffffc1c00001ffff87661d8ffff1e7ffffe;
5'd11: stable_row=700'h70700f3ffe7ffe38003c003ffefe3807383fff100000000000387ffffe0e0f703ffffc0e00001c00387661d0ffff1e70000e;
5'd12: stable_row=700'h70703ff3fe7ffe38003c0001c0fe38073c3fff0000000000007078001e7ffe70003c000e00001ffff8766dc4ffff1e70e1ce;
5'd13: stable_row=700'h3ff0703ff0707fff3c003c0001c0383fff3c01e00000006000007078001e7ffdfe003c000700001ffff8766dce3c071e70efce;
5'd14: stable_row=700'h3ff0703ff7ff03873c003c00ffff383fff3cffffc00000f00000700000007ffdfe003c00070000000000766dde3c271e00f780;
5'd15: stable_row=700'h3ff0703387ff03873e003c00ffff383fff3effffdf0000f00000e07ffffe7001fe003c00038000fffffe766dfc3c3ffe00f620;
5'd16: stable_row=700'h70703380000387be003c00ffff3801e03e00001f0000600000e07ffffe7ffc787ffffe038000fffffe766dd83c7ffe00f0f0;
5'd17: stable_row=700'h7070fffbfe03823f003c001e00f801e03f3fff1f0000000000e07ffffe7ffcf87ffffe038000000000766dc83cf7fe00f3e0;
5'd18: stable_row=700'h7070fffbfe03803e003c001e00ff7fffbe3fff1c0000000000e00038007001fc7ffffe0380001ffff8766dc03ce71e0071c0;
5'd19: stable_row=700'h7070fffb0effffbe003c007fffff7fffbe38071c0000000000e00038707ffffc007e000380001c3c38766dc83de71e007180;
5'd20: stable_row=700'h70703383feffffba003c007fff3e7fffba3fff1c0000000000e03ff8707ffff600ff000380001ffff8766df83dc71e7ffffe;
5'd21: stable_row=700'h70703383feffffba003c007fff3803f8383fff1c0000000000e03ff8707ffef601ff800380001c3c38766ff83dc71e7ffffe;
5'd22: stable_row=700'h70703ff30e038038003c001e1c3807fc3838071c0000600000e03ff87803807603ffc00780001ffff8760e1c3c871e7ffffe;
5'd23: stable_row=700'h70703ff30e038038003c001e1c381ffe3838075c0000f000007000387833bb7607bde0070000003c0076161c3c071e007c00;
5'd24: stable_row=700'h70703ff3fe038038003c001e3c383defb83fff7c0000f00000700038fc753f720f3cf80700003ffffc763f1c3c07fe03fe00;
5'd25: stable_row=700'h70703383fe038038003c001e3838f9e7f83fff7c0000600000700039fc7c3f723e3c7c0e00003ffffc703b9c3c07fe3fffe0;
5'd26: stable_row=700'h7ffffe03830e0380387ffffe1e103871e3b838077c0000000000387ffffefc3b707c3c3e0e0000003c007073de3c07fe7f87fe;
5'd27: stable_row=700'h7ffffe03f30e0380387ffffe0fe03e21e0b83c073c0000000000387fffce6ffbf0783c1e1c0000ffffff7ee1ee1fe71e7c01fc;
5'd28: stable_row=700'h7ffffe03f3ee0380387ffffe0fe03e01e0383fc71c00000000001c7ffe0e0ff370203c04380000ffffff3ee1ce1fe71e200038;
5'd29: stable_row=700'h1f3ee03803800000007e01e01e0381fc70800000000000e00000407e070003c007000000000003e408c0fc000000000;
default: stable_row=0; endcase end endfunction
function [699:0] valid_row; input[4:0] row; begin case(row)
5'd6: valid_row=700'h670300e383800000000000000002003c0700038000180180000000000000000000000000000000018000071c0000000000000700004;
5'd7: valid_row=700'he70700e38381ffff81ffff87ffef003c07000387001801c000000000000000000000000000000003c000039fe1ffffc1ffff8707fde;
5'd8: valid_row=700'h1e70787ffe381ffff81ffff87ffef803c07600387001801c00000000000e000000000000700000003c001ff9fe1ffffc1ffff8767ffe;
5'd9: valid_row=700'h1c70387ffe381ffff81ffff87ffe7c7ffc760038700187fc00000000001c0000000000003800007ffffe1ffc7e1ffffc1c0038767ffc;
5'd10: valid_row=700'h78703c0e38ff0000001c003807803e7ffc763ffff83ff7fe0000000000380000000000001c00007ffffe1ffe701e00001ffff87661d8;
5'd11: valid_row=700'h7fff9c0e38ff0000001c003807801e7ffdfe3ffff83ff7fe0000000000380000000000000e00007ffffe0e0f701e00001c00387661d0;
5'd12: valid_row=700'h7fffde7fffff0000001ffff807804e03c1fe3ffffc3ff0ee0000000000700000000000000e000078001e7ffe701e00001ffff8766dc4;
5'd13: valid_row=700'hffce7fff380000001ffff8079ce203c1fe003c1c3980e40000600000700f0f3c0f0f3c07000078001e7ffdfe1e00381ffff8766dce;
5'd14: valid_row=700'he0ff01c0387ffffe1c0038079cf0fffe76003c1e3980e00000f00000701fdffc1fdffc0700000000007ffdfe1e0038000000766dde;
5'd15: valid_row=700'h70e0fe3ffe3c7ffffe1c0038079cf8fffe77003c1839cfff0000f00000e03ffffc3ffffc0380007ffffe7001fe1e0038fffffe766dfc;
5'd16: valid_row=700'h7ff07e3ffefc7ffffe1ffff87f9c78fffe72003c0039cfff0000600000e03e7e7c3e7e7c0380007ffffe7ffc781ffff8fffffe766dd8;
5'd17: valid_row=700'h7ffe7031cffe0038001ffff87f9c3c1c00707ffffe7fffff0000000000e03c3c3c3c3c3c0380007ffffe7ffcf81ffff8000000766dc8;
5'd18: valid_row=700'h1ffe3831cefe0438201ffff87f9c3e1c01f07ffffe7ff0e00000000000e03c3c3c3c3c3c0380000038007001fc1ffff81ffff8766dc0;
5'd19: valid_row=700'h38fe3c3ffebf0738e0000000079c3f7fffff7ffffe7ff6ee0000000000e03c3c3c3c3c3c0380000038707ffffc0000381c3c38766dc8;
5'd20: valid_row=700'h3ce1fe3ffe3f0f38f010e708079c3e7ffffe007e0003c6ee0000000000e03c3c3c3c3c3c0380003ff8707ffff60000381ffff8766df8;
5'd21: valid_row=700'h1fe0fe31ce3a1e387878e71e079c3e7ffe7e00fe0003c6ee0000000000e03c3c3c3c3c3c0380003ff8707ffef61000381c3c38766ff8;
5'd22: valid_row=700'hfc07e31ce3a1e38783ce77c079c3a1c187000ef0007e6ee0000000000e03c3c3c3c3c3c0780003ff878038076f000381ffff8760e1c;
5'd23: valid_row=700'h27c0063ffe383c383c1ee778079c381c3c7001e7800ee6ee0000e00000703c3c3c3c3c3c07000000387833bb76700038003c0076161c;
5'd24: valid_row=700'he3f0003ffe3878383e0ce730079c381c387003c3c01e76ee0000e00000703c3c3c3c3c3c0700000038fc753f727000383ffffc763f1c;
5'd25: valid_row=700'h73fff00c303878381e00e700079c381c78700f81f07c7ffe0000700000703c3c3c3c3c3c0e00000039fc7c3f727800783ffffc703b9c;
5'd26: valid_row=700'h77ffff3e7c38703c0e7ffffeffffb81e10701f00fc783ffe0000700000380000000000000e00007ffffefc3b707ffff8003c007073de;
5'd27: valid_row=700'h7f1ffe7e3f38103fc47ffffeffffb81fe0707e007e381ffe0000380000380000000000001c00007fffce6ffbf03ffff0ffffff7ee1ee;
5'd28: valid_row=700'h3f0606780f38003f807ffffeffffb80fc0703c003c20080000003800001c0000000000003800007ffe0e0ff3701fffe0ffffff3ee1ce;
5'd29: valid_row=700'h1c0000200238000f8000000000003807c07010000800000000000000000e00000000000070000000000407e0700000000000003e408c;
default: valid_row=0; endcase end endfunction
function [699:0] processing_row; input[4:0] row; begin case(row)
5'd6: processing_row=700'h3003800001e008000000003c0000000001c0700380700380c0;
5'd7: processing_row=700'h1ff8702389e001e01e000000003c007ffefe01c070038070019ffe;
5'd8: processing_row=700'h1ff838739ffe7fff9c000000003c007ffefe01c070ff80707fdffe;
5'd9: processing_row=700'h1ff9b87bbdfe7fffbc000000003c007ffefe01c038ff80707fc0c0;
5'd10: processing_row=700'h1c01b83bb87e01e038000000003c0071ce3801cff8ff83fe7feffe;
5'd11: processing_row=700'h1c01bc1390703fff100000007ffffe71ce3801cff80383fe38fffe;
5'd12: processing_row=700'h7ffd9c0380703fff000000007ffffe7ffe3809cffc7ffbfe39fcce;
5'd13: processing_row=700'h7ffd9c7ffc7001e0000000007ffffe7ffe381fce1c7ffb761fecce;
5'd14: processing_row=700'h7ffd9e7ffffeffffc0000000783c1e7ffe383dce1c7ffb761f8ffe;
5'd15: processing_row=700'h399e7ffffeffffdf000000783c1e71cefe39ce1e703b760f0ffe;
5'd16: processing_row=700'h3ff99f781ffe00001f000000783c1e71cefe79c71e73bb761f82f0;
5'd17: processing_row=700'h3ff99e781c783fff1f000000783c1e7ffefe71c73f73bb76ffeffc;
5'd18: processing_row=700'h3ffd9e7ffcf83fff1c000000783c1e7ffe3821c73f73bb7679fedf;
5'd19: processing_row=700'h39d9a7ffdfc38071c0000007ffffe7ffe3801c7be73bb76604806;
5'd20: processing_row=700'h38d9a7ffffc3fff1c0000007ffffe01c03801c3f273bb763ffffc;
5'd21: processing_row=700'h7ffd98781ffe3fff1c0000007ffffe7ffe3801c3f073bbf63ffffc;
5'd22: processing_row=700'h7ffd987ffd7638071c000000783c1e7ffff801c1e073bbf6003800;
5'd23: processing_row=700'h7ffd987ffc7638075c0000e0783c1e7ffef801c1e07ffbf61ff870;
5'd24: processing_row=700'h7c1987ffc723fff7c0000e0003c0001c0fc01c7f01fe0701ff870;
5'd25: processing_row=700'hfe198781c723fff7c000070003c0001c0bf003ff87ff070003870;
5'd26: processing_row=700'h3ef818781c7038077c000070003c00ffff0e7fff3e78fc70003870;
5'd27: processing_row=700'hfc7e187f1c703c073c000038003c00ffff027ffc1f707c707ffffe;
5'd28: processing_row=700'h783c183f1c703fc71c000038003c00ffff007fc00e2018707ffffe;
5'd29: processing_row=700'h200c181e1c701fc708000000003c00000000000004000000000000;
default: processing_row=0; endcase end endfunction
function [899:0] guide_row; input[4:0] row; begin case(row)
5'd6: guide_row=900'h301800402001e00000000000000000060000000e1800380ff0000000000000000000e01c02003800000000701000700000381c00001c00000000000000;
5'd7: guide_row=900'h783801e07801e00007fffe000000000f001fff8e1f80380fffff0000000000000000f01c070038000000107078007000003c1c3fff1c00000000000000;
5'd8: guide_row=900'h783c01f0f001e00007fffe000000000f001fff8e1ff3ff800fff01f8781f01e3f000e01c0fc0380000001c713c0070ff0ffe1c3fff1c03f0f03e03c7e0;
5'd9: guide_row=900'hf01c00f0f001e00007fffe0000001fffff9fff8e0ff3ff800f0007fe781f01e7fc3fff1c07e0380000003e739e0070ff0ffe1c3fff1c0ff8f03e03cff8;
5'd10: guide_row=900'h1fffdc0fffff1fffff87001e0000001fffff80003fc1f3ffbfffffc7fe781f03c7fe3fff1c03f8380000001ff70e3ffeff0fff1c1c381c1ff8f03e078ffc;
5'd11: guide_row=900'h1fffce0fffff1fffff87001e0000001fffff80003fc070863fffffcf863c3f03c41f3fff7f00fe3800000007ffffbffe380f07ff1c381c1f08787e07883e;
5'd12: guide_row=900'h1fffce0fffff1fffff87001e0000001e000780003fc071c7000f000f003c3f83c00f38077f003f3800000001ffff3ffe380787ff1c39fc1e00787f07801e;
5'd13: guide_row=900'h3c3cf03c3cf00000f0e0f0701e00007001e0000181e00079fffce0071ee07fffe0f003c3f83c00f38377f001e3800003010767f3c70381fffff1c39ff9e00787f07801e;
5'd14: guide_row=900'h7f7ff07f7ff03030f0e0f0701e00807001e00003c0000001fffcf3ff7ff87fffe0f001c3f87801f00781c00063800007838f0003c70381fff1c1c39ff8f00387f0f003e;
5'd15: guide_row=900'hfffff0fffff0f038f8fffff01e01e07fffe00003c1fffff9fffcf3ff7ff870f0e0f801e3987803e00381c0000380000783fe7fe3c78381fff1c1c381f07f03c730f007c;
5'd16: guide_row=900'hf9f9f0f9f9f0f038f8fffff01e03e07fffe0000181fffff80701f3ff7ff870f0e07c01e79c781fc7fff9c3fffff0000303fe7fe3c38381ce37c1c381c03f03cf38f03f8;
5'd17: guide_row=900'hf0f0f0f0f0f07078fcfffff01e07c07fffe0000001fffff80707f8e703807fffe03e01e79c703f87ffffc3fffff0000001fc7fe3c38381ce37c1c381c0ff03cf38e07f0;
5'd18: guide_row=900'hf0f0f0f0f0f07870fce0f0701e0f807071e000000000e0006733f8e73ff87fffe01f00e78c707e07ffffe3fffff00000000770e1c38381ce37f9c381c1f001cf18e0fc0;
5'd19: guide_row=900'hf0f0f0f0f0f07870f8e0f0701e1f000071e000000000e1c07772fce73ff870f0e00f80f70cf0f800e1c7f803c3800000010770e1c38381ce33f9c381c3c001ee19e1f00;
5'd20: guide_row=900'hf0f0f0f0f0f038f0e8fffff01e0e0000f1e0000000ffe1c0e738fce73ff870f0e003c0ff0ef0f000e0e1f0038380000001c77fe1c39b87fffdf9c3c1c3c001fe1de1e00;
5'd21: guide_row=900'hf0f0f0f0f0f038e0e8fffff01e040000e0e0000000ffe1c0e738e8e783807fffe001e07f0ee0f010f0e1d0078380000003e77fe1c3df87fffdc1c3c9c3c000fe1dc1e02;
5'd22: guide_row=900'hf0f0f0f0f0f03ce0e0fffff01e000001e0e0000000ffe1e1e738e8e393b07fffe001f07e07e0f83079f1c00f0380000001ff70e1c1dfe7fffdc1c3cdc3e040fc0fc1f06;
5'd23: guide_row=900'hf0f0f0f0f0f01de0e000f0001e000003c0f000038000e1e1c71ce0e3bbb000f000fff07e07e07ff07fe1c01e03800007007f70e1c1e7f81f01c1c1cfc1ffc0fc0fc0ffe;
5'd24: guide_row=900'hf0f0f0f0f0f01c60e3fffffc1e000007c0f000038000e3f1c71ce0e3bbb8fffff0fff07e07e07ff03f81c07c83800007001f7fe1c0e0f01f81c1c1efc0ffc0fc0fc0ffe;
5'd25: guide_row=900'hf0f0f0f0f0f01e00e3fffffc0e00001f80f00001c000e7f3c71ee0e3d3b8fffff0fff03e07c01fe07f01c1f8fb80000381077fe1c0f0307bc1c1e1f7e03f807c0f803fc;
5'd26: guide_row=900'h3fffee3fffffc0ffc007f00780001c1fffff88708e0e1c39c00f00000000000000001ffe1c3f0ffc00003878770e1fe7800f1f1c1fef1c00000000000000;
5'd27: guide_row=900'h3fffee000f0000ff801fe007c0000e1ffff3807e0e0e1e3f3fffffc00000000000003e3fdf3e03fc00001c38f70e1fc7c03e0fdf0fe7c800000000000000;
5'd28: guide_row=900'h3fffee000f00007f800f800300000e1fff83807e0e0e1e3e3fffffc00000000000003c1f9f18007800001c3fe3ee07c3803c07df07c78000000000000000;
5'd29: guide_row=900'he000f00000000060001000000000001003e0e0e081e0000000000000000000010030f0000180000003fe3ee00010010018f00020000000000000000;
5'd30: guide_row=900'h1fc00000000000000000000000000000000000;
default: guide_row=0; endcase end endfunction
function [899:0] oldguide_row; input[4:0] row; begin case(row)
5'd6: oldguide_row=900'h180038000000007800000700000e1c00000000000000780200701c00e078000000381c00001c00000000000000;
5'd7: oldguide_row=900'h1f80381fffff80780000078003ff0c1fffff80000000780f80701e00e07f8000003c1c3fff1c00000000000000;
5'd8: oldguide_row=900'h1ff3ff9fffff8078ff80078003ff8e1fffff80000000781f1ffbff00e07f81f00ffe1c3fff1c03f0f03e03c7e0;
5'd9: oldguide_row=900'hff3ff9fffff8078ff8ffffe03c1ce1fffff8000001fffce1fffff80e01f87fc0ffe1c3fff1c0ffcf03e03cff8;
5'd10: oldguide_row=900'h1f3ff800f001ffeff8ffffe01c1ee1e03878000001fffc8039e779fff1c0ffe0fff1c1c381c0ffcf03e078ffc;
5'd11: oldguide_row=900'h70860ffffe1ffe000ffffe1fffff1effc78000001fffc0038c739fff1c0f1e0f07ff1c381c1f0c787e07883e;
5'd12: oldguide_row=900'h71c70ffffe1ffe000f000e1fffef1efff780000000780007ffff9fffff8e0f0787ff1c39fc1e00787f07801e;
5'd13: oldguide_row=900'h71ee0ffffe1e38000f000e1c30ef1ef0ff80001800780107fffe1ce7ff9e0f1fffff1c39ff9e00787f07801e;
5'd14: oldguide_row=900'h3ff7ff8f0f0e1e39ffcffffe1c30ef9e79df80003c00780787000e1ce7ff9e0f1fff1c1c39ff9e00387f0f003e;
5'd15: oldguide_row=900'h3ff7ff8f0f0e1e39ffcffffe1fffef9e3f9780003c00780f87fffe1ce71c1e0f1fff1c1c381f1f003c730f007c;
5'd16: oldguide_row=900'h3ff7ff8ffffe1e39ffcffffe1fffef9e3fc780001800780607fffe1ce71e1f1f1ce37c1c381c0f803cf38f03f8;
5'd17: oldguide_row=900'he70380ffffe1e381e0f000e041f0fdfffff8000003fffe407000e1ce73e1ffe1ce37c1c381c07c03cf38e07f0;
5'd18: oldguide_row=900'he73ff8f0f0e0e380e0f000e0e3fcf9fe0ff8000003fffe007fffe1ce77f1ffc1ce37f9c381c03e01cf18e0fc0;
5'd19: oldguide_row=900'he73ff8f0f0e0e384e0f000e0fbbfe9e1fc78000003fffe407fffe1fffff1ef81ce33f9c381c01f01ee19e1f00;
5'd20: oldguide_row=900'he73ff8ffffe0e38ef0ffffe07fcfe9e7fc7800000001e1e07000e1fffff8e007fffdf9c3c1c00781fe1de1e00;
5'd21: oldguide_row=900'he78380ffffe0e38e70ffffe07f72e1e7807800000011e0e07fffe1fff5f8f007fffdc1c3c9c003c0fe1dc1e02;
5'd22: oldguide_row=900'he393b0ffffe0e3dc70ffffe077bce1e07f780000003cf0e07fffe1ce71dcf827fffdc1c3cdc003e0fc0fc1f06;
5'd23: oldguide_row=900'he3bbb0f0f180e3dc78f000e0efdfe1efff780003807878f00e07000e01d87fe01f01c1c1cfc1ffe0fc0fc0ffe;
5'd24: oldguide_row=900'he3bbb80077c0e1fff8f000e0eeeee1eff878000380787cf3fffffc0e01c83fe01f81c1c1efc1ffe0fc0fc0ffe;
5'd25: oldguide_row=900'he3d3b8007f80e1fffcf000e1ee7ae1e600780001c0f83c73fffffc0e01c80fe07bc1c1e1f7e1ffe07c0f803fc;
5'd26: oldguide_row=900'he1c39c03ff00fefff8ffffe3ce3ee1fffff80001c1fffe700e03c00e01c00000f1f1c1fef1c00000000000000;
5'd27: oldguide_row=900'he1e3f1fffff8fcf010ffffe187ffe1fffff80000e1fffe780e03e00e01c00003e0fdf0fe7c800000000000000;
5'd28: oldguide_row=900'he1e3e1ffeff87c7000ffffe087efe1fffff80000e1c7fc380e01e00e01c00003c07df07c78000000000000000;
5'd29: oldguide_row=900'he081e0fc00f0006000f000e003c2e1e00078000000c000200e00c00e01c000010018f00020000000000000000;
default: oldguide_row=0; endcase end endfunction
function [207:0] dimlabel_0; input[4:0] row; begin case(row)
5'd5: dimlabel_0=208'h7000e0400000000;
5'd6: dimlabel_0=208'hf00070e00000000;
5'd7: dimlabel_0=208'h7fffe039c00381e0f;
5'd8: dimlabel_0=208'h7fffe1fff80381e0f;
5'd9: dimlabel_0=208'h7000e1fff80383e0e;
5'd10: dimlabel_0=208'h70e01c03801c3f0e;
5'd11: dimlabel_0=208'h7fffe1c03801c3f1e;
5'd12: dimlabel_0=208'h7fffe1c03801c3f1c;
5'd13: dimlabel_0=208'h70e01c03801e679c;
5'd14: dimlabel_0=208'h1fff800e679c;
5'd15: dimlabel_0=208'h1fffc1fff800e6798;
5'd16: dimlabel_0=208'h1fffc0060000f63b8;
5'd17: dimlabel_0=208'h1801c20e6c007c3f8;
5'd18: dimlabel_0=208'h1871c31e6c007c3f8;
5'd19: dimlabel_0=208'h1871c7246e007c1f0;
5'd20: dimlabel_0=208'h7879c7606e007c1f0;
5'd21: dimlabel_0=208'he0fc0ee0e700381f0;
5'd22: dimlabel_0=208'h60ff06ffe60000000;
5'd23: dimlabel_0=208'h7fcff07fc00000000;
5'd24: dimlabel_0=208'h3fc1e000000000000;
default: dimlabel_0=0; endcase end endfunction
function [207:0] dimlabel_1; input[4:0] row; begin case(row)
5'd5: dimlabel_1=208'h1c003810000000;
5'd6: dimlabel_1=208'h3c001c38000000;
5'd7: dimlabel_1=208'h3ffffc0e7000381c;
5'd8: dimlabel_1=208'h3ffffc7ffe00381c;
5'd9: dimlabel_1=208'h7ffe00381c;
5'd10: dimlabel_1=208'h7ffe0700e00381c;
5'd11: dimlabel_1=208'h7ffe0700e00381c;
5'd12: dimlabel_1=208'h700e0700e00381c;
5'd13: dimlabel_1=208'h7ffe0700e003ffc;
5'd14: dimlabel_1=208'h7ffe07ffe003ffc;
5'd15: dimlabel_1=208'h7ffe003ffc;
5'd16: dimlabel_1=208'h1ffff8018000381c;
5'd17: dimlabel_1=208'h1ffff8839b00381c;
5'd18: dimlabel_1=208'h1c0038c79b00381c;
5'd19: dimlabel_1=208'h1cff39c91b80381c;
5'd20: dimlabel_1=208'h1cff39d81b80381c;
5'd21: dimlabel_1=208'h1cff3bb839c0381c;
5'd22: dimlabel_1=208'h1fff39bff9800000;
5'd23: dimlabel_1=208'hf80381ff0000000;
default: dimlabel_1=0; endcase end endfunction
function [207:0] dimlabel_2; input[4:0] row; begin case(row)
5'd5: dimlabel_2=208'h700000000060000000;
5'd6: dimlabel_2=208'hf003fffc0070000000;
5'd7: dimlabel_2=208'h7fffe3fffc00f00003fc;
5'd8: dimlabel_2=208'h7fffe00f0000600007fc;
5'd9: dimlabel_2=208'h7000e00f007fffc00ffc;
5'd10: dimlabel_2=208'h70e000f007fffc00e1c;
5'd11: dimlabel_2=208'h7fffe00f000060000e1c;
5'd12: dimlabel_2=208'h7fffe00f00006000071c;
5'd13: dimlabel_2=208'h70e0fffff00600007fc;
5'd14: dimlabel_2=208'hfffff00600007fc;
5'd15: dimlabel_2=208'h1fffc00f003fffc00ffc;
5'd16: dimlabel_2=208'h1fffc00f003fffc01e1c;
5'd17: dimlabel_2=208'h1801c00f000060001c1c;
5'd18: dimlabel_2=208'h1871c00f000060001e1c;
5'd19: dimlabel_2=208'h1871c00f000060001ffc;
5'd20: dimlabel_2=208'h7879c00f000060000ffc;
5'd21: dimlabel_2=208'he0fc000f0000600003fc;
5'd22: dimlabel_2=208'h60ff000f00fffff00000;
5'd23: dimlabel_2=208'h7fcff00f00fffff00000;
5'd24: dimlabel_2=208'h3fc1e00f000000000000;
default: dimlabel_2=0; endcase end endfunction
function [207:0] dimlabel_3; input[4:0] row; begin case(row)
5'd5: dimlabel_3=208'h8000000001c000000;
5'd6: dimlabel_3=208'h7fffe003c003ff0001c000000;
5'd7: dimlabel_3=208'h7fffe001e003ff0001c001fff;
5'd8: dimlabel_3=208'he020f0038707ffff01fff;
5'd9: dimlabel_3=208'h1ffee07878038707ffff01fff;
5'd10: dimlabel_3=208'h1806e0f03e03870000e0000e0;
5'd11: dimlabel_3=208'h1ffee1e01f0387000070000e0;
5'd12: dimlabel_3=208'h1806e3ffff3f87f00070000e0;
5'd13: dimlabel_3=208'h1ffee380ff3f87f00078000e0;
5'd14: dimlabel_3=208'he70000380071fff8000e0;
5'd15: dimlabel_3=208'h1ffee10000380071fff8000e0;
5'd16: dimlabel_3=208'h1ffee1fffe3800700e1c000e0;
5'd17: dimlabel_3=208'hf00e1fffe3800700e0c000e0;
5'd18: dimlabel_3=208'h3c061c00e3800700e0e000e0;
5'd19: dimlabel_3=208'h7fff71c00e3800700e07000e0;
5'd20: dimlabel_3=208'h7fff71c00e3800700e03000e0;
5'd21: dimlabel_3=208'h1c071c00e3800700e03000e0;
5'd22: dimlabel_3=208'h1c079fffe3ffff7fff900000;
5'd23: dimlabel_3=208'h1f831fffe3ffff7fff800000;
5'd24: dimlabel_3=208'hf821c00e380070000000000;
default: dimlabel_3=0; endcase end endfunction
function [207:0] dimlabel_4; input[4:0] row; begin case(row)
5'd5: dimlabel_4=208'h1c001c0000040001c0000000;
5'd6: dimlabel_4=208'h3c001e3f801e0001c0000000;
5'd7: dimlabel_4=208'h3ffffc3e3f800f0001c000381c;
5'd8: dimlabel_4=208'h3ffffc373b81078001c000381c;
5'd9: dimlabel_4=208'h73bb83c3c001c000381c;
5'd10: dimlabel_4=208'h7ffe0e39b8781f001c000381c;
5'd11: dimlabel_4=208'h7ffe1c1df8f00f8ffc000381c;
5'd12: dimlabel_4=208'h700e180ef9ffff8ffc000381c;
5'd13: dimlabel_4=208'h7ffe173cf9c07f801c000381c;
5'd14: dimlabel_4=208'h7ffe0739fb8000001c000381c;
5'd15: dimlabel_4=208'h739b88000001c000381c;
5'd16: dimlabel_4=208'h1ffff873bb8ffff001c000381c;
5'd17: dimlabel_4=208'h1ffff873bb8ffff001c000381c;
5'd18: dimlabel_4=208'h1c003873bb8e007001c0003c3c;
5'd19: dimlabel_4=208'h1cff3873bb8e007001c0001ff8;
5'd20: dimlabel_4=208'h1cff3873bf8e007001c0000ff0;
5'd21: dimlabel_4=208'h1cff3871df8e007001c00007e0;
5'd22: dimlabel_4=208'h1fff3871e38ffff1ffff800000;
5'd23: dimlabel_4=208'hf803870e38ffff1ffff800000;
5'd24: dimlabel_4=208'h70438e00700000000000;
default: dimlabel_4=0; endcase end endfunction
function [207:0] dimlabel_5; input[4:0] row; begin case(row)
5'd5: dimlabel_5=208'h1c001c000004000000000000;
5'd6: dimlabel_5=208'h3c001e3f801e01ffff800000;
5'd7: dimlabel_5=208'h3ffffc3e3f800f01ffff8007fc;
5'd8: dimlabel_5=208'h3ffffc373b81078001c0001ffc;
5'd9: dimlabel_5=208'h73bb83c3c001c0003ffc;
5'd10: dimlabel_5=208'h7ffe0e39b8781f005c0003e1c;
5'd11: dimlabel_5=208'h7ffe1c1df8f00f80fc000781c;
5'd12: dimlabel_5=208'h700e180ef9ffff81fc000701c;
5'd13: dimlabel_5=208'h7ffe173cf9c07f83dc000701c;
5'd14: dimlabel_5=208'h7ffe0739fb8000079c000701c;
5'd15: dimlabel_5=208'h739b88000071c000701c;
5'd16: dimlabel_5=208'h1ffff873bb8ffff0e1c000701c;
5'd17: dimlabel_5=208'h1ffff873bb8ffff1c1c000381c;
5'd18: dimlabel_5=208'h1c003873bb8e007081c0003e1c;
5'd19: dimlabel_5=208'h1cff3873bb8e007001c0001ffc;
5'd20: dimlabel_5=208'h1cff3873bf8e007001c0000ffc;
5'd21: dimlabel_5=208'h1cff3871df8e007001c00003fc;
5'd22: dimlabel_5=208'h1fff3871e38ffff001c0000000;
5'd23: dimlabel_5=208'hf803870e38ffff001c0000000;
5'd24: dimlabel_5=208'h70438e007001c0000000;
default: dimlabel_5=0; endcase end endfunction
function [207:0] dimlabel_6; input[4:0] row; begin case(row)
5'd5: dimlabel_6=208'h20660c00380000000;
5'd6: dimlabel_6=208'h3ffe77ffec00380000000;
5'd7: dimlabel_6=208'h3ffef7ffec0038000703c;
5'd8: dimlabel_6=208'h380fc0660c0038000703c;
5'd9: dimlabel_6=208'h3a2e83ffdfbffff00707c;
5'd10: dimlabel_6=208'hf7803ffdfbffff0070fc;
5'd11: dimlabel_6=208'h1e3c3366ce383870070fc;
5'd12: dimlabel_6=208'h3dde7bffce383870071dc;
5'd13: dimlabel_6=208'h11c4f366de383870073dc;
5'd14: dimlabel_6=208'h1c043ffff3838700739c;
5'd15: dimlabel_6=208'h3ffe03ffff3838700771c;
5'd16: dimlabel_6=208'h3ffe40001fbffff00771c;
5'd17: dimlabel_6=208'h3e0e3ffcdbffff007e1c;
5'd18: dimlabel_6=208'h7f0e3ffcd38387007c1c;
5'd19: dimlabel_6=208'h7f8e381cc38387007c1c;
5'd20: dimlabel_6=208'hddc63ffcc0038000781c;
5'd21: dimlabel_6=208'h39ce7381cc0038000781c;
5'd22: dimlabel_6=208'h71c773ffcc00380000000;
5'd23: dimlabel_6=208'h21c233ffcc00380000000;
5'd24: dimlabel_6=208'h1c02381cc00380000000;
default: dimlabel_6=0; endcase end endfunction
function [207:0] dimlabel_7; input[4:0] row; begin case(row)
5'd5: dimlabel_7=208'h70000000000000cc180070000000;
5'd6: dimlabel_7=208'hf00000007fffefffd80070000000;
5'd7: dimlabel_7=208'hfffff3fffc7fffefffd8007000380f;
5'd8: dimlabel_7=208'hfffff3fffc0e0e00cc180070001c0e;
5'd9: dimlabel_7=208'h3801c0e0e07ffbf7fffe01c0e;
5'd10: dimlabel_7=208'h1fff83801c0e0e07ffbf7fffe01c1e;
5'd11: dimlabel_7=208'h1fff83801c0e0e06cd9c7070e00e1c;
5'd12: dimlabel_7=208'h1c0383801c0e0e07ff9c7070e00e1c;
5'd13: dimlabel_7=208'h1fff83801c0e0e06cdbc7070e00e1c;
5'd14: dimlabel_7=208'h1fff83801cfffff7fffe7070e00738;
5'd15: dimlabel_7=208'h3801cfffff7fffe7070e00738;
5'd16: dimlabel_7=208'h7fffe3801c0e0e00003f7fffe00738;
5'd17: dimlabel_7=208'h7fffe3801c0e0e07ff9b7fffe003f0;
5'd18: dimlabel_7=208'h7000e3801c0e0707ff9a7070e003f0;
5'd19: dimlabel_7=208'h73fce3801c0e070703987070e003f0;
5'd20: dimlabel_7=208'h73fce3801c0e0787ff9800700001e0;
5'd21: dimlabel_7=208'h73fce3fffc0e03c7039800700001e0;
5'd22: dimlabel_7=208'h7ffce3fffc0e03e7ff980070000000;
5'd23: dimlabel_7=208'h3e00e3801c0e01e7ff980070000000;
5'd24: dimlabel_7=208'h3801c0e00c703980070000000;
default: dimlabel_7=0; endcase end endfunction
function [207:0] dimlabel_8; input[4:0] row; begin case(row)
5'd5: dimlabel_8=208'h1000f00007000070000000000;
5'd6: dimlabel_8=208'h3ffb010f08007000070000000000;
5'd7: dimlabel_8=208'h3ffb838f1c0070000380003003fc;
5'd8: dimlabel_8=208'h3c01c3cf3c007007fffe003c07fc;
5'd9: dimlabel_8=208'h1e00e0ef78007007fffe003f0ffc;
5'd10: dimlabel_8=208'hf80604f3000700001c0003f0f1c;
5'd11: dimlabel_8=208'h1fe2200f003ff00000e0003b0e1c;
5'd12: dimlabel_8=208'h7dff07fffe3ff00000e000380e1c;
5'd13: dimlabel_8=208'hf07b87fffe007000007000380f1c;
5'd14: dimlabel_8=208'h6019c00f00007003fff8003807fc;
5'd15: dimlabel_8=208'h1c00f00007003fff8003803fc;
5'd16: dimlabel_8=208'h7ff9e00f00007003807c003803fc;
5'd17: dimlabel_8=208'h7ff9ffffff007003807e0038079c;
5'd18: dimlabel_8=208'h381efffff007003807e00380f1c;
5'd19: dimlabel_8=208'h381c00f00007003807200380e1c;
5'd20: dimlabel_8=208'h381c00f00007003807000381e1c;
5'd21: dimlabel_8=208'h381c00f00007003807000383c1c;
5'd22: dimlabel_8=208'hfffdc00f007fffe3fff000000000;
5'd23: dimlabel_8=208'hfffdc00f007fffe3fff000000000;
5'd24: dimlabel_8=208'h1c00f00000003807000000000;
default: dimlabel_8=0; endcase end endfunction
function [207:0] dimlabel_9; input[4:0] row; begin case(row)
5'd5: dimlabel_9=208'h1000f00000000070000000000;
5'd6: dimlabel_9=208'h3ffb010f087fffe0070000000000;
5'd7: dimlabel_9=208'h3ffb838f1c7fffe00380003c03fc;
5'd8: dimlabel_9=208'h3c01c3cf3c007007fffe007f07fc;
5'd9: dimlabel_9=208'h1e00e0ef78007007fffe00ff0ffc;
5'd10: dimlabel_9=208'hf80604f3001700001c000e10f1c;
5'd11: dimlabel_9=208'h1fe2200f0003f00000e000e00e1c;
5'd12: dimlabel_9=208'h7dff07fffe07f00000e000e00e1c;
5'd13: dimlabel_9=208'hf07b87fffe0f7000007000f00f1c;
5'd14: dimlabel_9=208'h6019c00f001e7003fff8007807fc;
5'd15: dimlabel_9=208'h1c00f001c7003fff8003c03fc;
5'd16: dimlabel_9=208'h7ff9e00f00387003807c001e03fc;
5'd17: dimlabel_9=208'h7ff9ffffff707003807e000f079c;
5'd18: dimlabel_9=208'h381efffff207003807e00070f1c;
5'd19: dimlabel_9=208'h381c00f00007003807200ff8e1c;
5'd20: dimlabel_9=208'h381c00f00007003807000ff9e1c;
5'd21: dimlabel_9=208'h381c00f00007003807000ffbc1c;
5'd22: dimlabel_9=208'hfffdc00f00007003fff000000000;
5'd23: dimlabel_9=208'hfffdc00f00007003fff000000000;
5'd24: dimlabel_9=208'h1c00f00007003807000000000;
default: dimlabel_9=0; endcase end endfunction
function [15:0] digit_row; input[9:0] addr; begin case(addr)
10'd3: digit_row=16'hf0;
10'd4: digit_row=16'h3f8;
10'd5: digit_row=16'h30c;
10'd6: digit_row=16'h70c;
10'd7: digit_row=16'h706;
10'd8: digit_row=16'h7c6;
10'd9: digit_row=16'h6e6;
10'd10: digit_row=16'h676;
10'd11: digit_row=16'h61e;
10'd12: digit_row=16'h60e;
10'd13: digit_row=16'h70e;
10'd14: digit_row=16'h30c;
10'd15: digit_row=16'h1fc;
10'd16: digit_row=16'hf0;
10'd35: digit_row=16'h70;
10'd36: digit_row=16'h7c;
10'd37: digit_row=16'h6e;
10'd38: digit_row=16'h64;
10'd39: digit_row=16'h60;
10'd40: digit_row=16'h60;
10'd41: digit_row=16'h60;
10'd42: digit_row=16'h60;
10'd43: digit_row=16'h60;
10'd44: digit_row=16'h60;
10'd45: digit_row=16'h60;
10'd46: digit_row=16'h60;
10'd47: digit_row=16'h3fe;
10'd48: digit_row=16'h3fe;
10'd67: digit_row=16'hf0;
10'd68: digit_row=16'h1fc;
10'd69: digit_row=16'h388;
10'd70: digit_row=16'h300;
10'd71: digit_row=16'h300;
10'd72: digit_row=16'h300;
10'd73: digit_row=16'h180;
10'd74: digit_row=16'h180;
10'd75: digit_row=16'hc0;
10'd76: digit_row=16'h60;
10'd77: digit_row=16'h30;
10'd78: digit_row=16'h18;
10'd79: digit_row=16'h7fc;
10'd80: digit_row=16'h7fc;
10'd99: digit_row=16'hf8;
10'd100: digit_row=16'h1fc;
10'd101: digit_row=16'h384;
10'd102: digit_row=16'h300;
10'd103: digit_row=16'h300;
10'd104: digit_row=16'h180;
10'd105: digit_row=16'hf0;
10'd106: digit_row=16'h3f0;
10'd107: digit_row=16'h700;
10'd108: digit_row=16'h600;
10'd109: digit_row=16'h600;
10'd110: digit_row=16'h700;
10'd111: digit_row=16'h3fc;
10'd112: digit_row=16'hfc;
10'd131: digit_row=16'h1c0;
10'd132: digit_row=16'h1e0;
10'd133: digit_row=16'h1e0;
10'd134: digit_row=16'h1b0;
10'd135: digit_row=16'h198;
10'd136: digit_row=16'h18c;
10'd137: digit_row=16'h18c;
10'd138: digit_row=16'h186;
10'd139: digit_row=16'h183;
10'd140: digit_row=16'h7ff;
10'd141: digit_row=16'h7ff;
10'd142: digit_row=16'h180;
10'd143: digit_row=16'h180;
10'd144: digit_row=16'h180;
10'd163: digit_row=16'h3fc;
10'd164: digit_row=16'h3fc;
10'd165: digit_row=16'hc;
10'd166: digit_row=16'hc;
10'd167: digit_row=16'hc;
10'd168: digit_row=16'hc;
10'd169: digit_row=16'h1fc;
10'd170: digit_row=16'h3fc;
10'd171: digit_row=16'h700;
10'd172: digit_row=16'h600;
10'd173: digit_row=16'h600;
10'd174: digit_row=16'h300;
10'd175: digit_row=16'h3fc;
10'd176: digit_row=16'hfc;
10'd195: digit_row=16'h3e0;
10'd196: digit_row=16'h3f8;
10'd197: digit_row=16'h1c;
10'd198: digit_row=16'hc;
10'd199: digit_row=16'h6;
10'd200: digit_row=16'h1f6;
10'd201: digit_row=16'h3fe;
10'd202: digit_row=16'h70e;
10'd203: digit_row=16'h606;
10'd204: digit_row=16'h606;
10'd205: digit_row=16'h606;
10'd206: digit_row=16'h30c;
10'd207: digit_row=16'h3fc;
10'd208: digit_row=16'hf0;
10'd227: digit_row=16'h7fe;
10'd228: digit_row=16'h7fe;
10'd229: digit_row=16'h600;
10'd230: digit_row=16'h300;
10'd231: digit_row=16'h300;
10'd232: digit_row=16'h180;
10'd233: digit_row=16'h180;
10'd234: digit_row=16'hc0;
10'd235: digit_row=16'hc0;
10'd236: digit_row=16'h60;
10'd237: digit_row=16'h60;
10'd238: digit_row=16'h30;
10'd239: digit_row=16'h38;
10'd240: digit_row=16'h18;
10'd259: digit_row=16'h1f8;
10'd260: digit_row=16'h3fc;
10'd261: digit_row=16'h606;
10'd262: digit_row=16'h606;
10'd263: digit_row=16'h60e;
10'd264: digit_row=16'h39c;
10'd265: digit_row=16'h1f8;
10'd266: digit_row=16'h1f8;
10'd267: digit_row=16'h39c;
10'd268: digit_row=16'h606;
10'd269: digit_row=16'h606;
10'd270: digit_row=16'h70e;
10'd271: digit_row=16'h3fc;
10'd272: digit_row=16'h1f8;
10'd291: digit_row=16'hf0;
10'd292: digit_row=16'h3fc;
10'd293: digit_row=16'h30c;
10'd294: digit_row=16'h606;
10'd295: digit_row=16'h606;
10'd296: digit_row=16'h606;
10'd297: digit_row=16'h70e;
10'd298: digit_row=16'h7fc;
10'd299: digit_row=16'h6f8;
10'd300: digit_row=16'h600;
10'd301: digit_row=16'h300;
10'd302: digit_row=16'h380;
10'd303: digit_row=16'h1fc;
10'd304: digit_row=16'h7c;
10'd323: digit_row=16'hc06;
10'd324: digit_row=16'hc06;
10'd325: digit_row=16'hc06;
10'd326: digit_row=16'hc06;
10'd327: digit_row=16'hc06;
10'd328: digit_row=16'hc66;
10'd329: digit_row=16'hc66;
10'd330: digit_row=16'h4ec;
10'd331: digit_row=16'h4ec;
10'd332: digit_row=16'h6ac;
10'd333: digit_row=16'h7bc;
10'd334: digit_row=16'h79c;
10'd335: digit_row=16'h71c;
10'd336: digit_row=16'h71c;
10'd355: digit_row=16'h606;
10'd356: digit_row=16'h606;
10'd357: digit_row=16'h606;
10'd358: digit_row=16'h606;
10'd359: digit_row=16'h606;
10'd360: digit_row=16'h606;
10'd361: digit_row=16'h7fe;
10'd362: digit_row=16'h7fe;
10'd363: digit_row=16'h606;
10'd364: digit_row=16'h606;
10'd365: digit_row=16'h606;
10'd366: digit_row=16'h606;
10'd367: digit_row=16'h606;
10'd368: digit_row=16'h606;
10'd390: digit_row=16'h70;
10'd391: digit_row=16'h70;
10'd392: digit_row=16'h70;
10'd398: digit_row=16'h70;
10'd399: digit_row=16'h70;
10'd400: digit_row=16'h70;
10'd430: digit_row=16'h60;
10'd431: digit_row=16'hf0;
10'd432: digit_row=16'h60;
10'd454: digit_row=16'h336;
10'd455: digit_row=16'h7fe;
10'd456: digit_row=16'h6ee;
10'd457: digit_row=16'h666;
10'd458: digit_row=16'h666;
10'd459: digit_row=16'h666;
10'd460: digit_row=16'h666;
10'd461: digit_row=16'h666;
10'd462: digit_row=16'h666;
10'd463: digit_row=16'h666;
10'd464: digit_row=16'h666;
10'd515: digit_row=16'h1fc;
10'd516: digit_row=16'h3fc;
10'd517: digit_row=16'h70c;
10'd518: digit_row=16'h60c;
10'd519: digit_row=16'h60c;
10'd520: digit_row=16'h30c;
10'd521: digit_row=16'h1fc;
10'd522: digit_row=16'h3fc;
10'd523: digit_row=16'h70c;
10'd524: digit_row=16'h60c;
10'd525: digit_row=16'h60c;
10'd526: digit_row=16'h70c;
10'd527: digit_row=16'h3fc;
10'd528: digit_row=16'hfc;
10'd547: digit_row=16'h7fe;
10'd548: digit_row=16'h7fe;
10'd549: digit_row=16'h60;
10'd550: digit_row=16'h60;
10'd551: digit_row=16'h60;
10'd552: digit_row=16'h60;
10'd553: digit_row=16'h60;
10'd554: digit_row=16'h60;
10'd555: digit_row=16'h60;
10'd556: digit_row=16'h60;
10'd557: digit_row=16'h60;
10'd558: digit_row=16'h60;
10'd559: digit_row=16'h60;
10'd560: digit_row=16'h60;
10'd579: digit_row=16'h606;
10'd580: digit_row=16'h606;
10'd581: digit_row=16'h606;
10'd582: digit_row=16'h606;
10'd583: digit_row=16'h606;
10'd584: digit_row=16'h606;
10'd585: digit_row=16'h606;
10'd586: digit_row=16'h606;
10'd587: digit_row=16'h606;
10'd588: digit_row=16'h606;
10'd589: digit_row=16'h606;
10'd590: digit_row=16'h30e;
10'd591: digit_row=16'h3fc;
10'd592: digit_row=16'hf8;
10'd611: digit_row=16'hfe;
10'd612: digit_row=16'h1fe;
10'd613: digit_row=16'h386;
10'd614: digit_row=16'h306;
10'd615: digit_row=16'h606;
10'd616: digit_row=16'h606;
10'd617: digit_row=16'h606;
10'd618: digit_row=16'h606;
10'd619: digit_row=16'h606;
10'd620: digit_row=16'h606;
10'd621: digit_row=16'h306;
10'd622: digit_row=16'h386;
10'd623: digit_row=16'h1fe;
10'd624: digit_row=16'h7e;
10'd643: digit_row=16'h60e;
10'd644: digit_row=16'h60e;
10'd645: digit_row=16'h61e;
10'd646: digit_row=16'h616;
10'd647: digit_row=16'h636;
10'd648: digit_row=16'h636;
10'd649: digit_row=16'h666;
10'd650: digit_row=16'h666;
10'd651: digit_row=16'h6c6;
10'd652: digit_row=16'h6c6;
10'd653: digit_row=16'h686;
10'd654: digit_row=16'h786;
10'd655: digit_row=16'h706;
10'd656: digit_row=16'h706;
10'd675: digit_row=16'hc06;
10'd676: digit_row=16'hc06;
10'd677: digit_row=16'hc06;
10'd678: digit_row=16'h60c;
10'd679: digit_row=16'h60c;
10'd680: digit_row=16'h60c;
10'd681: digit_row=16'h318;
10'd682: digit_row=16'h318;
10'd683: digit_row=16'h318;
10'd684: digit_row=16'h1b0;
10'd685: digit_row=16'h1b0;
10'd686: digit_row=16'h1b0;
10'd687: digit_row=16'he0;
10'd688: digit_row=16'he0;
10'd714: digit_row=16'h1f8;
10'd715: digit_row=16'h1f8;
10'd739: digit_row=16'hfc;
10'd740: digit_row=16'h1fc;
10'd741: digit_row=16'h38c;
10'd742: digit_row=16'h30c;
10'd743: digit_row=16'h30c;
10'd744: digit_row=16'h38c;
10'd745: digit_row=16'h1fc;
10'd746: digit_row=16'h7c;
10'd747: digit_row=16'h1cc;
10'd748: digit_row=16'h18c;
10'd749: digit_row=16'h38c;
10'd750: digit_row=16'h30c;
10'd751: digit_row=16'h30c;
10'd752: digit_row=16'h60c;
default:digit_row=0;endcase end endfunction
function[699:0] sort_title;input[4:0] row;begin case(row)
5'd6:sort_title=700'h380e00204000e01800000000000000e02000e000;
5'd7:sort_title=700'h3c0e0079e000e33c0fffff00000020e0f000e000;
5'd8:sort_title=700'h1fffce00f1e000e71c0fffff0003f038e27800e1fe;
5'd9:sort_title=700'h1fffce00f0f000e79c0fffff000ff87ce73c00e1fe;
5'd10:sort_title=700'h1fffff81e07800e39e000f00001ff83fee1c7ffdfe;
5'd11:sort_title=700'he3f83c07c1fff8e000f00001f080fffff7ffc70;
5'd12:sort_title=700'he3f87c03e1fff8e000f00001e0003fffe7ffc70;
5'd13:sort_title=700'hff8e1f801f9fffcf000f00001e0020ecfe78e070;
5'd14:sort_title=700'hff8e3f000fc0e1cf000f00000f0071e00078e070;
5'd15:sort_title=700'hff8e1fffff80e1cf800f000007f07fcffc78f070;
5'd16:sort_title=700'he78e0bfffd00e1ef800f000003f07fcffc787070;
5'd17:sort_title=700'he3bf03fffc00e08fc00f00000ff03f8ffc787070;
5'd18:sort_title=700'h1fffbfc3c0e000e00f800f00001f0000ee1c387070;
5'd19:sort_title=700'h1fffbf83c0e03fffef800f00003c0020ee1c387070;
5'd20:sort_title=700'h1fff8f83c0e03fffee800f00003c0038effc387370;
5'd21:sort_title=700'h4e20e03c0f03fffee800f00003c007ceffc387bf0;
5'd22:sort_title=700'heef0e03c0f000e00e000f00003e043fee1c383bfc;
5'd23:sort_title=700'hee78e03c0f000e00e000f00001ffc0fee1c383cff;
5'd24:sort_title=700'h1ce3ce01c07800e00e000f00000ffc03effc381c1e;
5'd25:sort_title=700'h1ce3ce01c07c00e00e000f000003f820effc381e06;
5'd26:sort_title=700'h38fdcf81fe3f00e00e1fffff800000f0ee1c3fcf00;
5'd27:sort_title=700'h8fc8f81fe1f80e00e1fffff80000071ee1c3f8f80;
5'd28:sort_title=700'h7c0780fe0f00e00e1fffff8000007fc7dc0f8700;
5'd29:sort_title=700'h200e00e0000000000007fc7dc000200;
5'd30:sort_title=700'h3f8000000000;
default:sort_title=0;endcase end endfunction
function[899:0] sort_guide;input[4:0] row;begin case(row)
5'd5:sort_guide=900'h380030c30000000000000000000070001c0000e01c0000000e018060040000000008003800000000000000000000e060018000070300001c0000000000000;
5'd6:sort_guide=900'h380078c787ffff8000000000000070001e3f80e01c0000000e038667ce00001c0c1f003800000000000000000010e0e001800003830fff9c0000000000000;
5'd7:sort_guide=900'he03e007c01c0003c003cc707ffff800e03e03f00007803fe3f80e3fc000000ffe18667ce3c001c0c0ff7ff07c381f03c7e0000003ce270018ff0ffc30fff9c03f1c0f81e3f0;
5'd8:sort_guide=900'h1f07f81ff01f07ffff9ffffe7ffff801f07f83fc1ffffe3ff3f8ffbfe000000ffe1c76fc71c0ffffe01f7ff1ff381f03c7f8000001fe738018ff0ffc30fff9c07f9c0f81e3fc;
5'd9:sort_guide=900'h318ffc3ff01fc7ffff9ffffe00f0000318ffc3fe1ffffe3ff3b8ff86e0000000e04c36dc71c0ffffe0030ec3ff381f0787fc0000007ee3cfffff0e0efee1c1c0ff9c0f83c3fe;
5'd10:sort_guide=900'h318e3c3c301fc7ffff9c000e00f0000318e3c20f1ffffe383bb8ce0660000000e0ee36dd38e0000061ff0ee3c31c3f07843c0000013effefff1c070ffee1c1c0f08e1f83c21e;
5'd11:sort_guide=900'h319e1c38001cc000f01ffffe00f1c00319e1c0070001c01c3db8ce064000003fffe6061fb8e03ffc71ff3fe3801c3f87801c0000070fffcfff1c1ffffee1cfc0e00e1fc3c00e;
5'd12:sort_guide=900'h1f1c0e38001c00007003fff000f1c001f1c0e0030001c01e19b8ce0600001c3fff7f7ffdfcf03ffc7073bfe3801c3f87001c0003838edfce181c1ffe30e1cff8e00e1fc3800e;
5'd13:sort_guide=900'he1c0e38001c00007803807000f1c000e1c0e0f38001e008e1f8ce7ff0003e19ce7e7ffdf870381c78738e03801c3b8f007c0007c3fc000e1c1c18ce30e1cff8700e1dc7803e;
5'd14:sort_guide=900'h1c0e1c001c00003c0380703ff1c00001c0e3ff8000e07ef9f8ce7ff0003e19fe38071ce0703ffc7873fff1c00e738f01f80007c1f8ffce1c1c18cef0f1c1f87f0739c780fc;
5'd15:sort_guide=900'h1c0e1e001c03fffc03fff03ff1c00001c0e3ff8ffff07e79b9ff8600001c1dd318079ce0383ffc7871fff1e00e73cf03e000038000ffce1c1c18cefe71c1c03f0739e781f0;
5'd16:sort_guide=900'h1c0e0f801c03fffe03fff03ff1c00001c0e7878ffff0701bb9ff8600000001cf1c17dc703800007c710e00f80e73ce07800000010ee1ce1c1c18ce7e71c1c0ff0739e703c0;
5'd17:sort_guide=900'h1c0e07c01c03ffff000c0000f1c00001c0e7038ffff8701bb80e0600000001de7e3ffc701cffff780000007c0e71de0f000000038effce1c1c18ce3e71c9c1e00738ef0780;
5'd18:sort_guide=900'h1c0e01e01c03803f800c0000f1c00001c0e70380383c701b381e6660000001c87e76fdf81cffff787fff801e07e1fe0e00000001eeffce1cdc7fffb071cdc1c003f0ff0700;
5'd19:sort_guide=900'he1e00f01c03803b8ffffc00f1c00000e1e70380381c7efb381f666000003fff7e767df81ee007707fff800f07e1fc0e04000000fee1ce0efc7fffb070efc1c003f0fe0702;
5'd20:sort_guide=900'he1c00781c0380398ffffc00f1c00000e1c78700381e7efbb83f666000003fff06263c780e3ffe707003800787e0fc0f0c0000007effce0eff81e03070e7c1e043f07e0786;
5'd21:sort_guide=900'hffc3ff81c0380388e0c1c00f1c00000ffc3ff00380e701bf83be660003801f070061c000e3ffe707fff83ff87e0fc07fc0007001effc60f3f03f03070f3c0ffc3f07e03fe;
5'd22:sort_guide=900'h7f83ff81c0380380e0c1c00f1c000007f81fe0038067019f873e660003807fc7f061de0070380707fff83ff83c0fc07fc0007010ee1c7078307783070f1e07fc1e07e03fe;
5'd23:sort_guide=900'h1f03ff81c03fff80e0c1cfffffc00001f00f81ffff2701838f1ffe0001c0f3ffe001dfc07038070700383ff83c0f801f80003870ee1c7f7801f3e307f78c03f81e07c00fc;
5'd24:sort_guide=900'h3fff80fcc1cfffffc00000000001ffff07ff839e1ffe0001c3c0f067ffdf80383f8707fff800000000000000003830ee1c7e3e03e1fbc7f7c00000000000000;
5'd25:sort_guide=900'h3fff8078c1cfffffc00000000001ffff07ff839c0ffe0000e1803007ffc180381f0707fff800000000000000001c3fc7dc3e1c03c0f3c3f3800000000000000;
5'd26:sort_guide=900'h38038000c000000000000000000000000701838804000000e000000000000000070707003800000000000000001c1f879c0008010031c001000000000000000;
default:sort_guide=0;endcase end endfunction
function[319:0] sort_wait;input[4:0] row;begin case(row)
5'd5:sort_wait=320'h700c0000000380401e078;
5'd6:sort_wait=320'h1ffffc007000000000719c0ffffc0381c00e038;
5'd7:sort_wait=320'he03e007c1ffffc007000000000739c0ffffc7ffde1ffffc;
5'd8:sort_wait=320'h1f07f80fe1ffffc00700000000073ce0ffffc7ffcf1ffffc;
5'd9:sort_wait=320'h318ffc1ff001e0010700000000071ce001e007ffc78378ee;
5'd10:sort_wait=320'h318e3c1c7841e187877f8000007ffce001e0003803873fc6;
5'd11:sort_wait=320'h319e1c3838e1e3c7c77f8000007ffcf001e0003801021e44;
5'd12:sort_wait=320'h1f1c0e3838f1e783ef7f8000607ffef001e00fffe9000e00;
5'd13:sort_wait=320'he1c0e38387def01ff300000f0070ef801e00ffffc0ffffc;
5'd14:sort_wait=320'h1c0e3c783dee00ff380000f0070ef801e00ffffe0ffffc;
5'd15:sort_wait=320'h1c0e3ff009e6003f38000060070cfc01e000000e000e00;
5'd16:sort_wait=320'h1c0e3ff001e0003f3c0000000700fc01e001c00f3fffff;
5'd17:sort_wait=320'h1c0e3bc1ffffe07f1c000000fffef801e00fffefbfffff;
5'd18:sort_wait=320'h1c0e3801ffffe0f71e000000fffee801e00fffefc1c000;
5'd19:sort_wait=320'he1e1c01ffffe1e70e000000fffee001e00fffef9ffffe;
5'd20:sort_wait=320'he1c1e1001e003e70f0000600700e001e001c38e9ffffe;
5'd21:sort_wait=320'hffc0ff001e007c7078000f00700e001e001c78e01c0f0;
5'd22:sort_wait=320'h7f807f001e00787038000f00700e001e001c70e01c0e0;
5'd23:sort_wait=320'h1f003f001e00707818000600700e1ffffe1c20e01c1e0;
5'd24:sort_wait=320'h1e00207f88000000700e1ffffe1fc0e01fdc0;
5'd25:sort_wait=320'h1e00003f00000000700e1ffffe1fc0e01fc40;
5'd26:sort_wait=320'h1e00001f00000000700e0000000fc0e00fc00;
default:sort_wait=0;endcase end endfunction
function[319:0] sort_check;input[4:0] row;begin case(row)
5'd5:sort_check=320'he0e000000303070000000008003000038e0;
5'd6:sort_check=320'he1ff000003878703ff8fff1e00780001cfe;
5'd7:sort_check=320'he1fe070003838703ff8fff3c7ffff87fcfe;
5'd8:sort_check=320'he03e07c1ffc3c73bff8fff387ffff87fe3e;
5'd9:sort_check=320'h4ec3807f1ffcdc73b838e07387ffff870738;
5'd10:sort_check=320'heec3807f1ffdce73b838e071070003838738;
5'd11:sort_check=320'heefff0731c0fef73bff8e0700000001ffffe;
5'd12:sort_check=320'h1cefff0701c0ffe73bff8e073f7ffff9ffefe;
5'd13:sort_check=320'h38efff0701c047e73bff8e073f7ffff9c00fe;
5'd14:sort_check=320'h18e7380701c1070738000e073f7ffff9ffe38;
5'd15:sort_check=320'h4e73c0701c38387381c0e0738007001ffe7c;
5'd16:sort_check=320'h1ce67c0701c781c7381c0fff380071c1c00fc;
5'd17:sort_check=320'hee0fe0701cf1fe73bff8fff383ff0c1ffffe;
5'd18:sort_check=320'hfe3fe0701ce1fe73bff8fff383ff0e1ffebe;
5'd19:sort_check=320'h701bf0701c61fe73b8e0000b83ff0e00c03e;
5'd20:sort_check=320'h7c0ba0701c000c73b8e030cf80070e1ddf3a;
5'd21:sort_check=320'h3e03a0701c000073b8f079cf80071f1cdf3a;
5'd22:sort_check=320'h1f03a0700c01f0703878f1ef80073f1a1fb8;
5'd23:sort_check=320'hfc380700e01ff703ffcf0f3cffffb1f1db8;
5'd24:sort_check=320'h3f380000ff1fe3f1f39e07dc7fff3bf1db8;
5'd25:sort_check=320'h1f380000fe00e3f1f11c03887ffc19bf9b8;
5'd26:sort_check=320'h63800007e0003f0000801000000101f038;
default:sort_check=0;endcase end endfunction
function[319:0] sort_good_row;input[4:0] row;begin case(row)
5'd5:sort_good_row=320'h38003800000000700c0000000c30c000000;
5'd6:sort_good_row=320'h1ff9f8003800000000719c0ffffc1e31e1ffffe;
5'd7:sort_good_row=320'h380f801f00701ff9f8003c00000000739c0ffffc0f31c1ffffe;
5'd8:sort_good_row=320'h7c1fe07fc07c1ff9dc7ffff800000073ce0ffffc7ffff9ffffe;
5'd9:sort_good_row=320'hc63ff0ffc07f01c1dc7ffff800000071ce001e007ffff803c00;
5'd10:sort_good_row=320'hc638f0f0c07f01c1dc7ffff8000007ffce001e0070003803c00;
5'd11:sort_good_row=320'hc67870e000731fffdc000f00000007ffcf001e007ffff803c70;
5'd12:sort_good_row=320'h7c7038e000701fffde000700000607ffef001e000fffc003c70;
5'd13:sort_good_row=320'h387038e000701c3fde000780000f0070ef801e000e01c003c70;
5'd14:sort_good_row=320'h70387000701db9df0003c0000f0070ef801e000e01c0ffc70;
5'd15:sort_good_row=320'h70387800701db9df3fffc000060070cfc01e000fffc0ffc70;
5'd16:sort_good_row=320'h70383e00701db9de3fffe0000000700fc01e000fffc0ffc70;
5'd17:sort_good_row=320'h70381f00701db9de3ffff000000fffef801e0000300003c70;
5'd18:sort_good_row=320'h70380780701db9dc3803f800000fffee801e0000300003c70;
5'd19:sort_good_row=320'h387803c0701dbddc3803b800000fffee001e003ffff003c70;
5'd20:sort_good_row=320'h387001e0701dffdc380398000600700e001e003ffff003c70;
5'd21:sort_good_row=320'h3ff0ffe0701dffdc380388000f00700e001e0038307003c70;
5'd22:sort_good_row=320'h1fe0ffe07007e3dc380380000f00700e001e0038307003c70;
5'd23:sort_good_row=320'h7c0ffe0701ff9dc3fff80000600700e1ffffe383073fffff;
5'd24:sort_good_row=320'h3e7c9c3fff80000000700e1ffffe3f3073fffff;
5'd25:sort_good_row=320'h1c3c1c3fff80000000700e1ffffe1e3073fffff;
5'd26:sort_good_row=320'h8081c380380000000700e00000000300000000;
default:sort_good_row=0;endcase end endfunction
function[319:0] sort_bad_row;input[4:0] row;begin case(row)
5'd5:sort_bad_row=320'h38001c00000000700c00000001c0000e01c;
5'd6:sort_bad_row=320'h1ff9f8001c00000000719c0ffffc01e3f80e01c;
5'd7:sort_bad_row=320'he03e03f01ff9f8001e00000000739c0ffffc3fe3f80e3fc;
5'd8:sort_bad_row=320'h1f07f83fc1ff9dc7ffff800000073ce0ffffc3ff3f8ffbfe;
5'd9:sort_bad_row=320'h318ffc3fe01c1dc7ffff800000071ce001e003ff3b8ff86e;
5'd10:sort_bad_row=320'h318e3c20f01c1dc7ffff8000007ffce001e00383bb8ce066;
5'd11:sort_bad_row=320'h319e1c0071fffdc000700000007ffcf001e001c3db8ce064;
5'd12:sort_bad_row=320'h1f1c0e0031fffde000700000607ffef001e001e19b8ce060;
5'd13:sort_bad_row=320'he1c0e0f39c3fde000780000f0070ef801e0008e1f8ce7ff;
5'd14:sort_bad_row=320'h1c0e3ff9db9df000380000f0070ef801e007ef9f8ce7ff;
5'd15:sort_bad_row=320'h1c0e3ff9db9df3fffc000060070cfc01e007e79b9ff860;
5'd16:sort_bad_row=320'h1c0e7879db9de3fffc0000000700fc01e00701bb9ff860;
5'd17:sort_bad_row=320'h1c0e7039db9de3fffe000000fffef801e00701bb80e060;
5'd18:sort_bad_row=320'h1c0e7039db9dc00e0f000000fffee801e00701b381e666;
5'd19:sort_bad_row=320'he1e7039dbddc00e07000000fffee001e007efb381f666;
5'd20:sort_bad_row=320'he1c7871dffdc00e078000600700e001e007efbb83f666;
5'd21:sort_bad_row=320'hffc3ff1dffdc00e038000f00700e001e00701bf83be66;
5'd22:sort_bad_row=320'h7f81fe07e3dc00e018000f00700e001e007019f873e66;
5'd23:sort_bad_row=320'h1f00f81ff9dc7fffc8000600700e1ffffe701838f1ffe;
5'd24:sort_bad_row=320'h3e7c9c7fffc0000000700e1ffffe7ff839e1ffe;
5'd25:sort_bad_row=320'h1c3c1c7fffc0000000700e1ffffe7ff839c0ffe;
5'd26:sort_bad_row=320'h8081c000000000000700e00000070183880400;
default:sort_bad_row=0;endcase end endfunction
function[319:0] sort_return;input[4:0] row;begin case(row)
5'd5:sort_return=320'h780000000001c0000000;
5'd6:sort_return=320'h7801ffffe001e01ffffe;
5'd7:sort_return=320'he03e007c000000007801ffffe000e01ffffe;
5'd8:sort_return=320'h1f07f80fe000000007801ffffe7ffff9ffffe;
5'd9:sort_return=320'h318ffc1ff0000007ffff9c000e7ffff803c00;
5'd10:sort_return=320'h318e3c1c78000007ffff9c000e7ffff803c00;
5'd11:sort_return=320'h319e1c3838000007ffff9cffce00078003c70;
5'd12:sort_return=320'h1f1c0e383800060787879cffce03838003c70;
5'd13:sort_return=320'he1c0e3838000f0787879cffce0383c003c70;
5'd14:sort_return=320'h1c0e3c78000f0787879ce1ce0381e0ffc70;
5'd15:sort_return=320'h1c0e3ff000060787879ce1ce3ffde0ffc70;
5'd16:sort_return=320'h1c0e3ff000000787879ce1ce3ffcf0ffc70;
5'd17:sort_return=320'h1c0e3bc0000007ffff9ce1ce3ffcf803c70;
5'd18:sort_return=320'h1c0e3800000007ffff9cffce0380f803c70;
5'd19:sort_return=320'he1e1c00000007ffff9cffce0380f803c70;
5'd20:sort_return=320'he1c1e1000060787879cffce0380e803c70;
5'd21:sort_return=320'hffc0ff0000f0787879c000e0380e003c70;
5'd22:sort_return=320'h7f807f0000f0007801c000e0380e003c70;
5'd23:sort_return=320'h1f003f000060007801ffffe7ffee3fffff;
5'd24:sort_return=320'h7801ffffe7ffee3fffff;
5'd25:sort_return=320'h7801ffffe7ffee3fffff;
5'd26:sort_return=320'h7801c000e0000e000000;
default:sort_return=0;endcase end endfunction
function[319:0] sort_remove;input[4:0] row;begin case(row)
5'd5:sort_remove=320'he000700c0000000380401e078;
5'd6:sort_remove=320'h7ffff001e000719c0ffffc0381c00e038;
5'd7:sort_remove=320'h7ffff3fffff0739c0ffffc7ffde1ffffc;
5'd8:sort_remove=320'h7ffff3fffff073ce0ffffc7ffcf1ffffc;
5'd9:sort_remove=320'h70780040c0071ce001e007ffc78378ee;
5'd10:sort_remove=320'h70780ef3dc7ffce001e0003803873fc6;
5'd11:sort_remove=320'h70780e7f9c7ffcf001e0003801021e44;
5'd12:sort_remove=320'h70780e7f9c7ffef001e00fffe9000e00;
5'd13:sort_remove=320'h70780ef3dc070ef801e00ffffc0ffffc;
5'd14:sort_remove=320'h70780ec0dc070ef801e00ffffe0ffffc;
5'd15:sort_remove=320'hfffffcffffc070cfc01e000000e000e00;
5'd16:sort_remove=320'hfffffcffffc0700fc01e001c00f3fffff;
5'd17:sort_remove=320'hfffffc00e00fffef801e00fffefbfffff;
5'd18:sort_remove=320'h70381ffffcfffee801e00fffefc1c000;
5'd19:sort_remove=320'h70381ffffcfffee001e00fffef9ffffe;
5'd20:sort_remove=320'h70381c739c0700e001e001c38e9ffffe;
5'd21:sort_remove=320'h703c1ce1dc0700e001e001c78e01c0f0;
5'd22:sort_remove=320'h703c1cfffc0700e001e001c70e01c0e0;
5'd23:sort_remove=320'h701e1d8fdc0700e1ffffe1c20e01c1e0;
5'd24:sort_remove=320'h700f9c801c0700e1ffffe1fc0e01fdc0;
5'd25:sort_remove=320'h70078fc01c0700e1ffffe1fc0e01fc40;
5'd26:sort_remove=320'h70020fc01c0700e0000000fc0e00fc00;
default:sort_remove=0;endcase end endfunction
function[319:0] sort_pause;input[4:0] row;begin case(row)
5'd5:sort_pause=320'h2000e000000;
5'd6:sort_pause=320'h7ffff001c00000000070307c00e07fffc;
5'd7:sort_pause=320'h380f801f07ffff001c00000000070303fdffc7fffc;
5'd8:sort_pause=320'h7c1fe03f87ffff001c00000003ffff807dffc7fffc;
5'd9:sort_pause=320'hc63ff07fc00780041c00000003ffff800c3b070000;
5'd10:sort_pause=320'hc638f071e107861e1dfe000000000187fc3b870000;
5'd11:sort_pause=320'hc67870e0e3878f1f1dfe000000fff1c7fcff870000;
5'd12:sort_pause=320'h7c7038e0e3c79e0fbdfe000180fff1c1ceff87003c;
5'd13:sort_pause=320'h387038e0e1f7bc07fcc00003c0e071e1ce3807003c;
5'd14:sort_pause=320'h7038f1e0f7b803fce00003c0fff1e1cfffc7fffc;
5'd15:sort_pause=320'h7038ffc0279800fce0000180fff1e1c7ffc7fffc;
5'd16:sort_pause=320'h7038ffc0078000fcf00000000001f1c43807fffc;
5'd17:sort_pause=320'h7038ef07ffff81fc70000003fffde0000000003c;
5'd18:sort_pause=320'h7038e007ffff83dc78000003fffde1fffe0c003c;
5'd19:sort_pause=320'h38787007ffff879c38000003801dc1fffe3c003c;
5'd20:sort_pause=320'h3870784007800f9c3c000180fff9c1c00e1c003c;
5'd21:sort_pause=320'h3ff03fc007801f1c1e0003c0fff9c1fffe1c003c;
5'd22:sort_pause=320'h1fe01fc007801e1c0e0003c00e01c1fffe1e003c;
5'd23:sort_pause=320'h7c00fc007801c1e060001800e01c1c00e1ffff8;
5'd24:sort_pause=320'h780081fe20000000fe1c1fffe0ffff8;
5'd25:sort_pause=320'h780000fc000000007c1c1fffe07fff0;
5'd26:sort_pause=320'h7800007c000000001c1c1c00e000000;
default:sort_pause=0;endcase end endfunction
endmodule
module window3 #(parameter W=1024, B=8)(
 input clk,rst_n,vs,hs,de,input[10:0] x,y,input[B-1:0] din,
 output reg ov,oh,od,output reg[10:0] ox,oy,
 output reg[B-1:0] p0,p1,p2,p3,p4,p5,p6,p7,p8);
 reg[B-1:0] even_line[0:W-1],odd_line[0:W-1];
 reg[B-1:0] eq,oq,dq,t1,t2,m1,m2,b1,b2;
 reg vq,hq,enq;reg[10:0] xq,yq;
 wire[B-1:0] top=yq[0]?oq:eq,mid=yq[0]?eq:oq;
 always @(posedge clk) if(de && x<W) begin
  eq<=even_line[x];if(!y[0])even_line[x]<=din;
 end
 always @(posedge clk) if(de && x<W) begin
  oq<=odd_line[x];if(y[0])odd_line[x]<=din;
 end
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin
   vq<=0;hq<=0;enq<=0;xq<=0;yq<=0;dq<=0;
   ov<=0;oh<=0;od<=0;ox<=0;oy<=0;
   t1<=0;t2<=0;m1<=0;m2<=0;b1<=0;b2<=0;
   p0<=0;p1<=0;p2<=0;p3<=0;p4<=0;p5<=0;p6<=0;p7<=0;p8<=0;
  end else begin
   vq<=vs;hq<=hs;enq<=de;xq<=x;yq<=y;dq<=din;
   ov<=vq;oh<=hq;od<=enq;ox<=xq;oy<=yq;
   if(enq) begin
    p0<=t2;p1<=t1;p2<=top;p3<=m2;p4<=m1;p5<=mid;p6<=b2;p7<=b1;p8<=dq;
    t2<=t1;t1<=top;m2<=m1;m1<=mid;b2<=b1;b1<=dq;
   end else begin t1<=0;t2<=0;m1<=0;m2<=0;b1<=0;b2<=0;end
  end
 end
endmodule
module stream_algorithms #(parameter W=1024, LOW=80, HIGH=160)(
 input clk,rst_n,vs,hs,de,input[10:0] x,y,input[7:0] gray,
 output gv,gh,gd,output[10:0] gx,gy,output[7:0] gaussian,
 output mv,mh,md,output[10:0] mx,my,output opened,closed,
 output cv,ch,cd,output[10:0] cx,cy,output contour,
 output nv,nh,nd,output[10:0] nx,ny,output weak_edge,strong_edge,
 output ev,ed,output[10:0] ex,ey,output measurement_edge,output eh,output sobel_edge,
 output hv,hh,hd,output[10:0] hx,hy,output hole_foreground);
 wire av,ah,ad;wire[10:0] ax,ay;wire[7:0] a[0:8];
 window3 #(W,8) wa(clk,rst_n,vs,hs,de,x,y,gray,av,ah,ad,ax,ay,
 a[0],a[1],a[2],a[3],a[4],a[5],a[6],a[7],a[8]);
 wire[11:0] weighted={4'b0,a[0]}+{3'b0,a[1],1'b0}+{4'b0,a[2]}+
 {3'b0,a[3],1'b0}+{2'b0,a[4],2'b0}+{3'b0,a[5],1'b0}+
 {4'b0,a[6]}+{3'b0,a[7],1'b0}+{4'b0,a[8]};
 assign gaussian=ax>=2 && ay>=2 ? weighted[11:4] : 0;
 assign gv=av;assign gh=ah;assign gd=ad;assign gx=ax;assign gy=ay;
 wire[8:0] binary_window={a[8]>=128,a[7]>=128,a[6]>=128,a[5]>=128,a[4]>=128,a[3]>=128,a[2]>=128,a[1]>=128,a[0]>=128};
 wire erode=&binary_window,dilate=|binary_window;
 wire[1:0] b[0:8];
 wire[1:0] morph_first=ax>=2 && ay>=2 ? {dilate,erode} : 2'b0;
 window3 #(W,2) wm(clk,rst_n,av,ah,ad,ax,ay,morph_first,mv,mh,md,mx,my,
 b[0],b[1],b[2],b[3],b[4],b[5],b[6],b[7],b[8]);
 assign opened=mx>=4 && my>=4 && (b[0][0]|b[1][0]|b[2][0]|b[3][0]|b[4][0]|b[5][0]|b[6][0]|b[7][0]|b[8][0]);
 assign closed=mx>=4 && my>=4 && (b[0][1]&b[1][1]&b[2][1]&b[3][1]&b[4][1]&b[5][1]&b[6][1]&b[7][1]&b[8][1]);
 assign cv=av;assign ch=ah;assign cd=ad;assign cx=ax;assign cy=ay;
 assign contour=ax>=2 && ay>=2 && binary_window[4] && !erode;
 wire sv,sh,sd;wire[10:0] sx,sy;wire[7:0] s[0:8];
 window3 #(W,8) ws(clk,rst_n,gv,gh,gd,gx,gy,gaussian,sv,sh,sd,sx,sy,
 s[0],s[1],s[2],s[3],s[4],s[5],s[6],s[7],s[8]);
 wire[9:0] xp={2'b0,s[2]}+{1'b0,s[5],1'b0}+{2'b0,s[8]};
 wire[9:0] xn={2'b0,s[0]}+{1'b0,s[3],1'b0}+{2'b0,s[6]};
 wire[9:0] yp={2'b0,s[6]}+{1'b0,s[7],1'b0}+{2'b0,s[8]};
 wire[9:0] yn={2'b0,s[0]}+{1'b0,s[1],1'b0}+{2'b0,s[2]};
 wire[9:0] dx=xp>=xn?xp-xn:xn-xp,dy=yp>=yn?yp-yn:yn-yp;
 wire[10:0] magnitude={1'b0,dx}+{1'b0,dy};
 reg rv,rh,rd;reg[10:0] rx,ry,gradient;reg[7:0] center_gray;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin rv<=0;rh<=0;rd<=0;rx<=0;ry<=0;gradient<=0;center_gray<=0;end
  else begin rv<=sv;rh<=sh;rd<=sd;rx<=sx;ry<=sy;
   center_gray<=s[4];
   gradient<=sx>=4 && sy>=4 ? magnitude : 0;
  end
 end
 assign ev=rv;assign eh=rh;assign ed=rd;assign ex=rx;assign ey=ry;
 // Gaussian suppresses texture noise; no bright-side gate that loses dark edges.
 assign sobel_edge=gradient>=96;
 // Robust 20th/95th percentiles on a 4x4 subsample, excluding OSD.
 reg[15:0] histogram[0:63];
 reg gv_old,scanning,contrast_ok;reg[6:0] bin_index;
 reg[15:0] cumulative;reg[5:0] low_bin;reg low_found;
 reg[7:0] object_threshold;integer k;
 wire[16:0] cumulative_next={1'b0,cumulative}+histogram[bin_index[5:0]];
 wire[6:0] midpoint_bins={1'b0,low_bin}+{1'b0,bin_index[5:0]};
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   gv_old<=0;scanning<=0;bin_index<=0;cumulative<=0;low_bin<=0;low_found<=0;
   object_threshold<=255;contrast_ok<=0;
   for(k=0;k<64;k=k+1)histogram[k]<=0;
  end else begin
   gv_old<=gv;
   if(gv && !gv_old)begin scanning<=1;bin_index<=0;cumulative<=0;low_found<=0;contrast_ok<=0;end
   else if(scanning)begin
    cumulative<=cumulative_next;histogram[bin_index[5:0]]<=0;
    if(!low_found && cumulative_next>=4840)begin low_bin<=bin_index[5:0];low_found<=1;end
    if(cumulative<22990 && cumulative_next>=22990 && low_found)begin
     object_threshold<={midpoint_bins,1'b0}+8'd2;
     contrast_ok<=bin_index>=low_bin+2;
    end
    if(bin_index==63)scanning<=0;else bin_index<=bin_index+1'b1;
   end else if(gd && gx>=160 && gx<960 && gy>=64 && gy<548 && gx[1:0]==0 && gy[1:0]==0)
    histogram[gaussian[7:2]]<=histogram[gaussian[7:2]]+1'b1;
  end
 end
 // Silhouette support supplements Sobel when the camera edge contrast is low.
 assign measurement_edge=contrast_ok && center_gray>=object_threshold;
 // Unused legacy ports, retained for module interface compatibility only.
 assign nv=rv;assign nh=rh;assign nd=rd;assign nx=rx;assign ny=ry;
 assign weak_edge=sobel_edge;assign strong_edge=sobel_edge;
 assign hv=0;assign hh=0;assign hd=0;assign hx=0;assign hy=0;assign hole_foreground=0;
endmodule
module controls_measure #(parameter CLK_HZ=68493000)(
 input clk,rst_n,key_function_n,key_action_n,frame_vs,
 input edge_vs,edge_de,input[10:0] edge_x,edge_y,input edge_pixel,
 output reg[1:0] function_id,output reg[3:0] algorithm,
 output reg calibrated,output reg[8:0] span,
 output wire[15:0] width10,height10,
 output wire[10:0] bx,by,bw,bh,output wire[1:0] measure_status,
 output[127:0] extra10,output[7:0] feature_valid,output[87:0] landmarks,
 output reg sort_run,output reg sort_rearm,output[19:0] sort_gain_x,sort_gain_y);
 localparam DB=CLK_HZ/50,HOLD=CLK_HZ;
 reg[1:0] fk,ak;reg fs,as_;reg[31:0] fc,ac,held;reg long_done;
 reg[1:0] requested_function;reg[3:0] requested_algorithm;reg fv,ev,ed_old;
 wire[19:0] gain=scale_gain(span),gain_y=scale_gain_y(span);
 assign sort_gain_x=gain;assign sort_gain_y=gain_y;
 wire[159:0] dimensions;
 wire[10:0] mbx,mby,mbw,mbh;wire[1:0] ms;
 assign width10=dimensions[15:0];assign height10=dimensions[31:16];
 assign bx=mbx;assign by=mby;assign bw=mbw;assign bh=mbh;assign measure_status=ms;
 profile_measure profile(clk,rst_n,calibrated,edge_vs,edge_de,edge_x,edge_y,edge_pixel,
 gain,gain_y,dimensions,feature_valid,mbx,mby,mbw,mbh,ms,landmarks);
 assign extra10=dimensions[159:32];
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n)begin
   fk<=3;ak<=3;fs<=1;as_<=1;fc<=0;ac<=0;held<=0;long_done<=0;
   requested_function<=0;requested_algorithm<=0;function_id<=0;algorithm<=0;
   calibrated<=0;span<=160;fv<=0;ev<=0;ed_old<=0;sort_run<=1;sort_rearm<=0;
  end else begin
   sort_rearm<=0;
   fk<={fk[0],key_function_n};ak<={ak[0],key_action_n};fv<=frame_vs;ev<=edge_vs;ed_old<=edge_de;
   if(fk[1]==fs)fc<=0;
   else if(fc==DB-1)begin
    fc<=0;fs<=fk[1];if(!fk[1])requested_function<=requested_function==2?0:requested_function+1'b1;
   end else fc<=fc+1'b1;
   if(ak[1]==as_)ac<=0;
   else if(ac==DB-1)begin
    ac<=0;as_<=ak[1];
    if(ak[1] && !long_done)begin
     if(function_id==0)requested_algorithm<=requested_algorithm==7?0:requested_algorithm+1'b1;
     else if(!calibrated)span<=span>=320?80:span+2;
     else if(function_id==2)begin sort_run<=~sort_run;if(!sort_run)sort_rearm<=1;end
    end
   end else ac<=ac+1'b1;
   if(as_)begin held<=0;long_done<=0;end
   else if(!long_done)begin
    if(held==HOLD-1)begin
     held<=0;long_done<=1;
     if(function_id!=0)begin calibrated<=~calibrated;end
    end else held<=held+1'b1;
   end
   if(frame_vs && !fv)begin
    function_id<=requested_function;algorithm<=requested_algorithm;
    if(function_id!=requested_function && !as_)begin held<=0;long_done<=1;end
   end
  end
 end
// Reciprocal functions appended by generate.py.
function[19:0] scale_gain;input[8:0] span;begin case(span)
9'd80:scale_gain=20'd16384;
9'd81:scale_gain=20'd16182;
9'd82:scale_gain=20'd15984;
9'd83:scale_gain=20'd15792;
9'd84:scale_gain=20'd15604;
9'd85:scale_gain=20'd15420;
9'd86:scale_gain=20'd15241;
9'd87:scale_gain=20'd15066;
9'd88:scale_gain=20'd14895;
9'd89:scale_gain=20'd14727;
9'd90:scale_gain=20'd14564;
9'd91:scale_gain=20'd14404;
9'd92:scale_gain=20'd14247;
9'd93:scale_gain=20'd14094;
9'd94:scale_gain=20'd13944;
9'd95:scale_gain=20'd13797;
9'd96:scale_gain=20'd13653;
9'd97:scale_gain=20'd13513;
9'd98:scale_gain=20'd13375;
9'd99:scale_gain=20'd13240;
9'd100:scale_gain=20'd13107;
9'd101:scale_gain=20'd12977;
9'd102:scale_gain=20'd12850;
9'd103:scale_gain=20'd12725;
9'd104:scale_gain=20'd12603;
9'd105:scale_gain=20'd12483;
9'd106:scale_gain=20'd12365;
9'd107:scale_gain=20'd12250;
9'd108:scale_gain=20'd12136;
9'd109:scale_gain=20'd12025;
9'd110:scale_gain=20'd11916;
9'd111:scale_gain=20'd11808;
9'd112:scale_gain=20'd11703;
9'd113:scale_gain=20'd11599;
9'd114:scale_gain=20'd11498;
9'd115:scale_gain=20'd11398;
9'd116:scale_gain=20'd11299;
9'd117:scale_gain=20'd11203;
9'd118:scale_gain=20'd11108;
9'd119:scale_gain=20'd11014;
9'd120:scale_gain=20'd10923;
9'd121:scale_gain=20'd10832;
9'd122:scale_gain=20'd10744;
9'd123:scale_gain=20'd10656;
9'd124:scale_gain=20'd10570;
9'd125:scale_gain=20'd10486;
9'd126:scale_gain=20'd10403;
9'd127:scale_gain=20'd10321;
9'd128:scale_gain=20'd10240;
9'd129:scale_gain=20'd10161;
9'd130:scale_gain=20'd10082;
9'd131:scale_gain=20'd10005;
9'd132:scale_gain=20'd9930;
9'd133:scale_gain=20'd9855;
9'd134:scale_gain=20'd9781;
9'd135:scale_gain=20'd9709;
9'd136:scale_gain=20'd9638;
9'd137:scale_gain=20'd9567;
9'd138:scale_gain=20'd9498;
9'd139:scale_gain=20'd9430;
9'd140:scale_gain=20'd9362;
9'd141:scale_gain=20'd9296;
9'd142:scale_gain=20'd9230;
9'd143:scale_gain=20'd9166;
9'd144:scale_gain=20'd9102;
9'd145:scale_gain=20'd9039;
9'd146:scale_gain=20'd8978;
9'd147:scale_gain=20'd8916;
9'd148:scale_gain=20'd8856;
9'd149:scale_gain=20'd8797;
9'd150:scale_gain=20'd8738;
9'd151:scale_gain=20'd8680;
9'd152:scale_gain=20'd8623;
9'd153:scale_gain=20'd8567;
9'd154:scale_gain=20'd8511;
9'd155:scale_gain=20'd8456;
9'd156:scale_gain=20'd8402;
9'd157:scale_gain=20'd8349;
9'd158:scale_gain=20'd8296;
9'd159:scale_gain=20'd8244;
9'd160:scale_gain=20'd8192;
9'd161:scale_gain=20'd8141;
9'd162:scale_gain=20'd8091;
9'd163:scale_gain=20'd8041;
9'd164:scale_gain=20'd7992;
9'd165:scale_gain=20'd7944;
9'd166:scale_gain=20'd7896;
9'd167:scale_gain=20'd7849;
9'd168:scale_gain=20'd7802;
9'd169:scale_gain=20'd7756;
9'd170:scale_gain=20'd7710;
9'd171:scale_gain=20'd7665;
9'd172:scale_gain=20'd7620;
9'd173:scale_gain=20'd7576;
9'd174:scale_gain=20'd7533;
9'd175:scale_gain=20'd7490;
9'd176:scale_gain=20'd7447;
9'd177:scale_gain=20'd7405;
9'd178:scale_gain=20'd7364;
9'd179:scale_gain=20'd7322;
9'd180:scale_gain=20'd7282;
9'd181:scale_gain=20'd7242;
9'd182:scale_gain=20'd7202;
9'd183:scale_gain=20'd7162;
9'd184:scale_gain=20'd7123;
9'd185:scale_gain=20'd7085;
9'd186:scale_gain=20'd7047;
9'd187:scale_gain=20'd7009;
9'd188:scale_gain=20'd6972;
9'd189:scale_gain=20'd6935;
9'd190:scale_gain=20'd6899;
9'd191:scale_gain=20'd6862;
9'd192:scale_gain=20'd6827;
9'd193:scale_gain=20'd6791;
9'd194:scale_gain=20'd6756;
9'd195:scale_gain=20'd6722;
9'd196:scale_gain=20'd6687;
9'd197:scale_gain=20'd6653;
9'd198:scale_gain=20'd6620;
9'd199:scale_gain=20'd6587;
9'd200:scale_gain=20'd6554;
9'd201:scale_gain=20'd6521;
9'd202:scale_gain=20'd6489;
9'd203:scale_gain=20'd6457;
9'd204:scale_gain=20'd6425;
9'd205:scale_gain=20'd6394;
9'd206:scale_gain=20'd6363;
9'd207:scale_gain=20'd6332;
9'd208:scale_gain=20'd6302;
9'd209:scale_gain=20'd6271;
9'd210:scale_gain=20'd6242;
9'd211:scale_gain=20'd6212;
9'd212:scale_gain=20'd6183;
9'd213:scale_gain=20'd6154;
9'd214:scale_gain=20'd6125;
9'd215:scale_gain=20'd6096;
9'd216:scale_gain=20'd6068;
9'd217:scale_gain=20'd6040;
9'd218:scale_gain=20'd6012;
9'd219:scale_gain=20'd5985;
9'd220:scale_gain=20'd5958;
9'd221:scale_gain=20'd5931;
9'd222:scale_gain=20'd5904;
9'd223:scale_gain=20'd5878;
9'd224:scale_gain=20'd5851;
9'd225:scale_gain=20'd5825;
9'd226:scale_gain=20'd5800;
9'd227:scale_gain=20'd5774;
9'd228:scale_gain=20'd5749;
9'd229:scale_gain=20'd5724;
9'd230:scale_gain=20'd5699;
9'd231:scale_gain=20'd5674;
9'd232:scale_gain=20'd5650;
9'd233:scale_gain=20'd5625;
9'd234:scale_gain=20'd5601;
9'd235:scale_gain=20'd5578;
9'd236:scale_gain=20'd5554;
9'd237:scale_gain=20'd5530;
9'd238:scale_gain=20'd5507;
9'd239:scale_gain=20'd5484;
9'd240:scale_gain=20'd5461;
9'd241:scale_gain=20'd5439;
9'd242:scale_gain=20'd5416;
9'd243:scale_gain=20'd5394;
9'd244:scale_gain=20'd5372;
9'd245:scale_gain=20'd5350;
9'd246:scale_gain=20'd5328;
9'd247:scale_gain=20'd5307;
9'd248:scale_gain=20'd5285;
9'd249:scale_gain=20'd5264;
9'd250:scale_gain=20'd5243;
9'd251:scale_gain=20'd5222;
9'd252:scale_gain=20'd5201;
9'd253:scale_gain=20'd5181;
9'd254:scale_gain=20'd5160;
9'd255:scale_gain=20'd5140;
9'd256:scale_gain=20'd5120;
9'd257:scale_gain=20'd5100;
9'd258:scale_gain=20'd5080;
9'd259:scale_gain=20'd5061;
9'd260:scale_gain=20'd5041;
9'd261:scale_gain=20'd5022;
9'd262:scale_gain=20'd5003;
9'd263:scale_gain=20'd4984;
9'd264:scale_gain=20'd4965;
9'd265:scale_gain=20'd4946;
9'd266:scale_gain=20'd4928;
9'd267:scale_gain=20'd4909;
9'd268:scale_gain=20'd4891;
9'd269:scale_gain=20'd4873;
9'd270:scale_gain=20'd4855;
9'd271:scale_gain=20'd4837;
9'd272:scale_gain=20'd4819;
9'd273:scale_gain=20'd4801;
9'd274:scale_gain=20'd4784;
9'd275:scale_gain=20'd4766;
9'd276:scale_gain=20'd4749;
9'd277:scale_gain=20'd4732;
9'd278:scale_gain=20'd4715;
9'd279:scale_gain=20'd4698;
9'd280:scale_gain=20'd4681;
9'd281:scale_gain=20'd4664;
9'd282:scale_gain=20'd4648;
9'd283:scale_gain=20'd4632;
9'd284:scale_gain=20'd4615;
9'd285:scale_gain=20'd4599;
9'd286:scale_gain=20'd4583;
9'd287:scale_gain=20'd4567;
9'd288:scale_gain=20'd4551;
9'd289:scale_gain=20'd4535;
9'd290:scale_gain=20'd4520;
9'd291:scale_gain=20'd4504;
9'd292:scale_gain=20'd4489;
9'd293:scale_gain=20'd4473;
9'd294:scale_gain=20'd4458;
9'd295:scale_gain=20'd4443;
9'd296:scale_gain=20'd4428;
9'd297:scale_gain=20'd4413;
9'd298:scale_gain=20'd4398;
9'd299:scale_gain=20'd4384;
9'd300:scale_gain=20'd4369;
9'd301:scale_gain=20'd4355;
9'd302:scale_gain=20'd4340;
9'd303:scale_gain=20'd4326;
9'd304:scale_gain=20'd4312;
9'd305:scale_gain=20'd4297;
9'd306:scale_gain=20'd4283;
9'd307:scale_gain=20'd4269;
9'd308:scale_gain=20'd4256;
9'd309:scale_gain=20'd4242;
9'd310:scale_gain=20'd4228;
9'd311:scale_gain=20'd4215;
9'd312:scale_gain=20'd4201;
9'd313:scale_gain=20'd4188;
9'd314:scale_gain=20'd4174;
9'd315:scale_gain=20'd4161;
9'd316:scale_gain=20'd4148;
9'd317:scale_gain=20'd4135;
9'd318:scale_gain=20'd4122;
9'd319:scale_gain=20'd4109;
9'd320:scale_gain=20'd4096;
default:scale_gain=20'd8192;endcase end endfunction
function[19:0] scale_gain_y;input[8:0] span;begin case(span)
9'd80:scale_gain_y=20'd15729;
9'd81:scale_gain_y=20'd15534;
9'd82:scale_gain_y=20'd15345;
9'd83:scale_gain_y=20'd15160;
9'd84:scale_gain_y=20'd14980;
9'd85:scale_gain_y=20'd14803;
9'd86:scale_gain_y=20'd14631;
9'd87:scale_gain_y=20'd14463;
9'd88:scale_gain_y=20'd14299;
9'd89:scale_gain_y=20'd14138;
9'd90:scale_gain_y=20'd13981;
9'd91:scale_gain_y=20'd13827;
9'd92:scale_gain_y=20'd13677;
9'd93:scale_gain_y=20'd13530;
9'd94:scale_gain_y=20'd13386;
9'd95:scale_gain_y=20'd13245;
9'd96:scale_gain_y=20'd13107;
9'd97:scale_gain_y=20'd12972;
9'd98:scale_gain_y=20'd12840;
9'd99:scale_gain_y=20'd12710;
9'd100:scale_gain_y=20'd12583;
9'd101:scale_gain_y=20'd12458;
9'd102:scale_gain_y=20'd12336;
9'd103:scale_gain_y=20'd12216;
9'd104:scale_gain_y=20'd12099;
9'd105:scale_gain_y=20'd11984;
9'd106:scale_gain_y=20'd11871;
9'd107:scale_gain_y=20'd11760;
9'd108:scale_gain_y=20'd11651;
9'd109:scale_gain_y=20'd11544;
9'd110:scale_gain_y=20'd11439;
9'd111:scale_gain_y=20'd11336;
9'd112:scale_gain_y=20'd11235;
9'd113:scale_gain_y=20'd11135;
9'd114:scale_gain_y=20'd11038;
9'd115:scale_gain_y=20'd10942;
9'd116:scale_gain_y=20'd10847;
9'd117:scale_gain_y=20'd10755;
9'd118:scale_gain_y=20'd10663;
9'd119:scale_gain_y=20'd10574;
9'd120:scale_gain_y=20'd10486;
9'd121:scale_gain_y=20'd10399;
9'd122:scale_gain_y=20'd10314;
9'd123:scale_gain_y=20'd10230;
9'd124:scale_gain_y=20'd10148;
9'd125:scale_gain_y=20'd10066;
9'd126:scale_gain_y=20'd9986;
9'd127:scale_gain_y=20'd9908;
9'd128:scale_gain_y=20'd9830;
9'd129:scale_gain_y=20'd9754;
9'd130:scale_gain_y=20'd9679;
9'd131:scale_gain_y=20'd9605;
9'd132:scale_gain_y=20'd9533;
9'd133:scale_gain_y=20'd9461;
9'd134:scale_gain_y=20'd9390;
9'd135:scale_gain_y=20'd9321;
9'd136:scale_gain_y=20'd9252;
9'd137:scale_gain_y=20'd9185;
9'd138:scale_gain_y=20'd9118;
9'd139:scale_gain_y=20'd9052;
9'd140:scale_gain_y=20'd8988;
9'd141:scale_gain_y=20'd8924;
9'd142:scale_gain_y=20'd8861;
9'd143:scale_gain_y=20'd8799;
9'd144:scale_gain_y=20'd8738;
9'd145:scale_gain_y=20'd8678;
9'd146:scale_gain_y=20'd8618;
9'd147:scale_gain_y=20'd8560;
9'd148:scale_gain_y=20'd8502;
9'd149:scale_gain_y=20'd8445;
9'd150:scale_gain_y=20'd8389;
9'd151:scale_gain_y=20'd8333;
9'd152:scale_gain_y=20'd8278;
9'd153:scale_gain_y=20'd8224;
9'd154:scale_gain_y=20'd8171;
9'd155:scale_gain_y=20'd8118;
9'd156:scale_gain_y=20'd8066;
9'd157:scale_gain_y=20'd8015;
9'd158:scale_gain_y=20'd7964;
9'd159:scale_gain_y=20'd7914;
9'd160:scale_gain_y=20'd7864;
9'd161:scale_gain_y=20'd7815;
9'd162:scale_gain_y=20'd7767;
9'd163:scale_gain_y=20'd7720;
9'd164:scale_gain_y=20'd7673;
9'd165:scale_gain_y=20'd7626;
9'd166:scale_gain_y=20'd7580;
9'd167:scale_gain_y=20'd7535;
9'd168:scale_gain_y=20'd7490;
9'd169:scale_gain_y=20'd7446;
9'd170:scale_gain_y=20'd7402;
9'd171:scale_gain_y=20'd7358;
9'd172:scale_gain_y=20'd7316;
9'd173:scale_gain_y=20'd7273;
9'd174:scale_gain_y=20'd7232;
9'd175:scale_gain_y=20'd7190;
9'd176:scale_gain_y=20'd7149;
9'd177:scale_gain_y=20'd7109;
9'd178:scale_gain_y=20'd7069;
9'd179:scale_gain_y=20'd7030;
9'd180:scale_gain_y=20'd6991;
9'd181:scale_gain_y=20'd6952;
9'd182:scale_gain_y=20'd6914;
9'd183:scale_gain_y=20'd6876;
9'd184:scale_gain_y=20'd6839;
9'd185:scale_gain_y=20'd6802;
9'd186:scale_gain_y=20'd6765;
9'd187:scale_gain_y=20'd6729;
9'd188:scale_gain_y=20'd6693;
9'd189:scale_gain_y=20'd6658;
9'd190:scale_gain_y=20'd6623;
9'd191:scale_gain_y=20'd6588;
9'd192:scale_gain_y=20'd6554;
9'd193:scale_gain_y=20'd6520;
9'd194:scale_gain_y=20'd6486;
9'd195:scale_gain_y=20'd6453;
9'd196:scale_gain_y=20'd6420;
9'd197:scale_gain_y=20'd6387;
9'd198:scale_gain_y=20'd6355;
9'd199:scale_gain_y=20'd6323;
9'd200:scale_gain_y=20'd6291;
9'd201:scale_gain_y=20'd6260;
9'd202:scale_gain_y=20'd6229;
9'd203:scale_gain_y=20'd6198;
9'd204:scale_gain_y=20'd6168;
9'd205:scale_gain_y=20'd6138;
9'd206:scale_gain_y=20'd6108;
9'd207:scale_gain_y=20'd6079;
9'd208:scale_gain_y=20'd6049;
9'd209:scale_gain_y=20'd6021;
9'd210:scale_gain_y=20'd5992;
9'd211:scale_gain_y=20'd5963;
9'd212:scale_gain_y=20'd5935;
9'd213:scale_gain_y=20'd5907;
9'd214:scale_gain_y=20'd5880;
9'd215:scale_gain_y=20'd5853;
9'd216:scale_gain_y=20'd5825;
9'd217:scale_gain_y=20'd5799;
9'd218:scale_gain_y=20'd5772;
9'd219:scale_gain_y=20'd5746;
9'd220:scale_gain_y=20'd5720;
9'd221:scale_gain_y=20'd5694;
9'd222:scale_gain_y=20'd5668;
9'd223:scale_gain_y=20'd5643;
9'd224:scale_gain_y=20'd5617;
9'd225:scale_gain_y=20'd5592;
9'd226:scale_gain_y=20'd5568;
9'd227:scale_gain_y=20'd5543;
9'd228:scale_gain_y=20'd5519;
9'd229:scale_gain_y=20'd5495;
9'd230:scale_gain_y=20'd5471;
9'd231:scale_gain_y=20'd5447;
9'd232:scale_gain_y=20'd5424;
9'd233:scale_gain_y=20'd5400;
9'd234:scale_gain_y=20'd5377;
9'd235:scale_gain_y=20'd5354;
9'd236:scale_gain_y=20'd5332;
9'd237:scale_gain_y=20'd5309;
9'd238:scale_gain_y=20'd5287;
9'd239:scale_gain_y=20'd5265;
9'd240:scale_gain_y=20'd5243;
9'd241:scale_gain_y=20'd5221;
9'd242:scale_gain_y=20'd5200;
9'd243:scale_gain_y=20'd5178;
9'd244:scale_gain_y=20'd5157;
9'd245:scale_gain_y=20'd5136;
9'd246:scale_gain_y=20'd5115;
9'd247:scale_gain_y=20'd5094;
9'd248:scale_gain_y=20'd5074;
9'd249:scale_gain_y=20'd5053;
9'd250:scale_gain_y=20'd5033;
9'd251:scale_gain_y=20'd5013;
9'd252:scale_gain_y=20'd4993;
9'd253:scale_gain_y=20'd4973;
9'd254:scale_gain_y=20'd4954;
9'd255:scale_gain_y=20'd4934;
9'd256:scale_gain_y=20'd4915;
9'd257:scale_gain_y=20'd4896;
9'd258:scale_gain_y=20'd4877;
9'd259:scale_gain_y=20'd4858;
9'd260:scale_gain_y=20'd4840;
9'd261:scale_gain_y=20'd4821;
9'd262:scale_gain_y=20'd4803;
9'd263:scale_gain_y=20'd4784;
9'd264:scale_gain_y=20'd4766;
9'd265:scale_gain_y=20'd4748;
9'd266:scale_gain_y=20'd4730;
9'd267:scale_gain_y=20'd4713;
9'd268:scale_gain_y=20'd4695;
9'd269:scale_gain_y=20'd4678;
9'd270:scale_gain_y=20'd4660;
9'd271:scale_gain_y=20'd4643;
9'd272:scale_gain_y=20'd4626;
9'd273:scale_gain_y=20'd4609;
9'd274:scale_gain_y=20'd4592;
9'd275:scale_gain_y=20'd4576;
9'd276:scale_gain_y=20'd4559;
9'd277:scale_gain_y=20'd4543;
9'd278:scale_gain_y=20'd4526;
9'd279:scale_gain_y=20'd4510;
9'd280:scale_gain_y=20'd4494;
9'd281:scale_gain_y=20'd4478;
9'd282:scale_gain_y=20'd4462;
9'd283:scale_gain_y=20'd4446;
9'd284:scale_gain_y=20'd4431;
9'd285:scale_gain_y=20'd4415;
9'd286:scale_gain_y=20'd4400;
9'd287:scale_gain_y=20'd4384;
9'd288:scale_gain_y=20'd4369;
9'd289:scale_gain_y=20'd4354;
9'd290:scale_gain_y=20'd4339;
9'd291:scale_gain_y=20'd4324;
9'd292:scale_gain_y=20'd4309;
9'd293:scale_gain_y=20'd4295;
9'd294:scale_gain_y=20'd4280;
9'd295:scale_gain_y=20'd4265;
9'd296:scale_gain_y=20'd4251;
9'd297:scale_gain_y=20'd4237;
9'd298:scale_gain_y=20'd4222;
9'd299:scale_gain_y=20'd4208;
9'd300:scale_gain_y=20'd4194;
9'd301:scale_gain_y=20'd4180;
9'd302:scale_gain_y=20'd4167;
9'd303:scale_gain_y=20'd4153;
9'd304:scale_gain_y=20'd4139;
9'd305:scale_gain_y=20'd4126;
9'd306:scale_gain_y=20'd4112;
9'd307:scale_gain_y=20'd4099;
9'd308:scale_gain_y=20'd4085;
9'd309:scale_gain_y=20'd4072;
9'd310:scale_gain_y=20'd4059;
9'd311:scale_gain_y=20'd4046;
9'd312:scale_gain_y=20'd4033;
9'd313:scale_gain_y=20'd4020;
9'd314:scale_gain_y=20'd4007;
9'd315:scale_gain_y=20'd3995;
9'd316:scale_gain_y=20'd3982;
9'd317:scale_gain_y=20'd3969;
9'd318:scale_gain_y=20'd3957;
9'd319:scale_gain_y=20'd3944;
9'd320:scale_gain_y=20'd3932;
default:scale_gain_y=20'd7864;endcase end endfunction
endmodule
// Atomic decimal conversion outside the per-pixel OSD combinational path.
// One digit correction/shift per clock; result updates every 18 clocks.
module bin_to_bcd(input clk,rst_n,input[15:0] value,output reg[19:0] digits);
 reg[15:0] bits;
 reg[19:0] work;
 reg[4:0] step;
 reg[19:0] adjusted;
 integer i;
 always @* begin
  adjusted=work;
  for(i=0;i<5;i=i+1)
   if(work[i*4+:4]>=5)adjusted[i*4+:4]=work[i*4+:4]+4'd3;
 end
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin bits<=0;work<=0;step<=0;digits<=0;end
  else if(step==0)begin bits<=value;work<=0;step<=1;end
  else if(step<=16)begin
   work<={adjusted[18:0],bits[15]};bits<={bits[14:0],1'b0};step<=step+1'b1;
  end else begin digits<=work;step<=0;end
 end
endmodule
// Fixed orientation, one bright silhouette. Profile RAM is scanned in blanking.
module profile_measure(input clk,rst_n,calibrated,vs,de,input[10:0] x,y,input foreground,
 input[19:0] gain_x,gain_y,output reg[159:0] dimensions,output reg[7:0] feature_valid,
 output reg[10:0] bx,by,bw,bh,output reg[1:0] status,output reg[87:0] landmarks);
 reg prev_vs,prev_de;reg[10:0] line_y,run_start,run_len,best_start,best_len;
 reg[10:0] xmin,xmax,ymin,ymax;reg[9:0] rows;reg clipped;
 wire take_run=run_len>best_len;
 wire[10:0] row_len=take_run?run_len:best_len,row_left=take_run?run_start:best_start;
 wire[10:0] row_right=row_left+row_len-1'b1;
 wire valid_row=row_len>=24;
 reg[22:0] profile[0:599];reg[22:0] q;
 reg[9:0] address,scan_y;reg[3:0] state;
 reg[10:0] top_y,bottom_y,left_x,right_x,body_left,tab_first,tab_last,notch_first,notch_last,notch_min;
 reg tab_seen,notch_seen;reg[2:0] stable;
 reg[10:0] pixels[0:9];reg[3:0] scale_index,scale_delayed;reg[31:0] product;
 wire vertical=scale_index==1 || scale_index==3 || scale_index==4 || scale_index==5 || scale_index==7;
 reg arc_side,mean_side;
 reg[10:0] arc_dx,arc_dy;
 reg[21:0] square_x,square_y,radicand;
 reg[19:0] radius_sum[0:1];reg[7:0] radius_count[0:1];
 reg[10:0] radius_min[0:1],radius_max[0:1];
 wire sqrt_done,mean_done;wire[10:0] radius;wire[19:0] mean_radius;
 wire upper_zone=scan_y>=top_y+3 && scan_y<top_y+((bottom_y-top_y)>>2);
 wire lower_zone=scan_y+3<=bottom_y && scan_y>bottom_y-((bottom_y-top_y)>>2);
 wire[10:0] arc_distance=upper_zone?scan_y-top_y:bottom_y-scan_y;
 sqrt22 root_calc(clk,rst_n,state==8,radicand,sqrt_done,radius);
 wire[19:0] rounded_sum=radius_sum[mean_side]+(radius_count[mean_side]>>1);
 mean20 mean_calc(clk,rst_n,state==10,rounded_sum,radius_count[mean_side],mean_done,mean_radius);
 integer j;
 // Synchronous read, one write per row. All 600 entries refreshed every frame.
 always @(posedge clk)begin
  if(prev_de && !de)profile[line_y]<={valid_row,row_left,row_right};
  q<=profile[address];
 end
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   prev_vs<=0;prev_de<=0;line_y<=0;run_start<=0;run_len<=0;best_start<=0;best_len<=0;
   xmin<=1023;xmax<=0;ymin<=599;ymax<=0;rows<=0;clipped<=0;
   address<=0;scan_y<=0;state<=0;top_y<=0;bottom_y<=0;left_x<=0;right_x<=0;body_left<=0;
   tab_first<=0;tab_last<=0;notch_first<=0;notch_last<=0;notch_min<=0;tab_seen<=0;notch_seen<=0;
   bx<=0;by<=0;bw<=0;bh<=0;status<=0;stable<=0;feature_valid<=0;
   scale_index<=0;scale_delayed<=0;product<=0;dimensions<=0;landmarks<=0;
   arc_side<=0;mean_side<=0;arc_dx<=0;arc_dy<=0;square_x<=0;square_y<=0;radicand<=0;
   radius_sum[0]<=0;radius_sum[1]<=0;radius_count[0]<=0;radius_count[1]<=0;
   radius_min[0]<=2047;radius_min[1]<=2047;radius_max[0]<=0;radius_max[1]<=0;
   for(j=0;j<10;j=j+1)pixels[j]<=0;
  end else begin
   prev_vs<=vs;prev_de<=de;
   scale_index<=scale_index==9?0:scale_index+1'b1;scale_delayed<=scale_index;
   product<=pixels[scale_index]*(vertical?gain_y:gain_x);
   dimensions[scale_delayed*16+:16]<=(product*10+32768)>>16;
   if(!calibrated)begin status<=0;feature_valid<=0;stable<=0;end
   if(de)begin
    line_y<=y;
    if(x>=160 && x<960 && y>=64 && y<548 && foreground)begin
     if(run_len==0)run_start<=x;
     run_len<=run_len+1'b1;
    end else begin
     if(run_len>best_len)begin best_len<=run_len;best_start<=run_start;end
     run_len<=0;
    end
   end
   if(prev_de && !de)begin
    if(valid_row)begin
     rows<=rows+1'b1;
     if(row_left<xmin)xmin<=row_left;if(row_right>xmax)xmax<=row_right;
     if(line_y<ymin)ymin<=line_y;if(line_y>ymax)ymax<=line_y;
     if(row_left==160 || row_right==959 || line_y==64 || line_y==547)clipped<=1;
    end
    run_len<=0;best_len<=0;best_start<=0;
   end
   if(vs && !prev_vs)begin
    rows<=0;xmin<=1023;xmax<=0;ymin<=599;ymax<=0;clipped<=0;state<=0;
    if(!calibrated || rows<24 || clipped || xmax-xmin<24 || ymax-ymin<24)begin
     status<=0;stable<=0;feature_valid<=0;
    end else begin
     bx<=xmin-2;by<=ymin-2;bw<=xmax-xmin+1'b1;bh<=ymax-ymin+1'b1;
     pixels[0]<=xmax-xmin+1'b1;pixels[1]<=ymax-ymin+1'b1;
     if(status!=0 && xmin+6>=bx && bx+10>=xmin && ymin+6>=by && by+10>=ymin &&
      xmax-xmin+9>=bw && bw+7>=xmax-xmin && ymax-ymin+9>=bh && bh+7>=ymax-ymin)begin
      if(stable<3)stable<=stable+1'b1;status<=stable>=2?2:1;
     end else begin stable<=0;status<=1;end
     top_y<=ymin;bottom_y<=ymax;left_x<=xmin;right_x<=xmax;
     address<=ymin+((ymax-ymin)>>2);state<=1;feature_valid<=0;
    end
   end else case(state)
    1:state<=2;
    2:begin
     // The quarter-height row intersects the straight left edge of the body.
     body_left<=q[21:11];address<=top_y;scan_y<=top_y;
     tab_seen<=0;notch_seen<=0;notch_min<=right_x;
     radius_sum[0]<=0;radius_sum[1]<=0;radius_count[0]<=0;radius_count[1]<=0;
     radius_min[0]<=2047;radius_min[1]<=2047;radius_max[0]<=0;radius_max[1]<=0;
     if(q[22])begin pixels[2]<=right_x-q[21:11]+1'b1;state<=3;end else state<=0;
    end
    3:state<=4;
    4:begin
     if(q[22])begin
      if(q[21:11]+((right_x-body_left)>>2)<body_left)begin
       if(!tab_seen)tab_first<=scan_y;tab_seen<=1;tab_last<=scan_y;
      end
      // Only central half: excludes the two R7 corner recesses.
      if(scan_y>=top_y+((bottom_y-top_y)>>2) && scan_y<=bottom_y-((bottom_y-top_y)>>2) && q[10:0]+2<right_x)begin
       if(!notch_seen)notch_first<=scan_y;notch_seen<=1;notch_last<=scan_y;
       if(q[10:0]<notch_min)notch_min<=q[10:0];
      end
     end
     if(q[22] && q[10:0]+3<right_x && (upper_zone || lower_zone))begin
      arc_side<=lower_zone;arc_dx<=right_x-q[10:0];arc_dy<=normalize_y(arc_distance);state<=6;
     end else if(scan_y==bottom_y)state<=5;
     else begin scan_y<=scan_y+1'b1;address<=scan_y+1'b1;state<=3;end
    end
    5:begin
     feature_valid[0]<=body_left>left_x+12 && right_x>body_left+24;
     if(tab_seen)begin
      pixels[3]<=tab_last-tab_first+1'b1;pixels[4]<=tab_first-top_y;pixels[5]<=bottom_y-tab_last;
     end
     feature_valid[3:1]<={3{tab_seen && tab_first>top_y && tab_last<bottom_y}};
     if(notch_seen)begin pixels[6]<=right_x-notch_min;pixels[7]<=notch_last-notch_first+1'b1;end
     feature_valid[5:4]<={2{notch_seen && right_x>notch_min+3}};
     landmarks[10:0]<=body_left-2;landmarks[21:11]<=tab_first-2;landmarks[32:22]<=tab_last-2;
     landmarks[43:33]<=notch_min-2;landmarks[54:44]<=notch_first-2;landmarks[65:55]<=notch_last-2;
     mean_side<=0;state<=10;
    end
    6:begin square_x<=arc_dx*arc_dx;square_y<=arc_dy*arc_dy;state<=7;end
    7:begin radicand<=square_x+square_y;state<=8;end
    8:state<=9;
    9:if(sqrt_done)begin
     radius_sum[arc_side]<=radius_sum[arc_side]+radius;radius_count[arc_side]<=radius_count[arc_side]+1'b1;
     if(radius<radius_min[arc_side])radius_min[arc_side]<=radius;
     if(radius>radius_max[arc_side])radius_max[arc_side]<=radius;
     if(scan_y==bottom_y)state<=5;
     else begin scan_y<=scan_y+1'b1;address<=scan_y+1'b1;state<=3;end
    end
    10:state<=11;
    11:if(mean_done)begin
     pixels[mean_side?9:8]<=mean_radius[10:0];
     landmarks[(mean_side?77:66)+:11]<=mean_radius[10:0];
     feature_valid[mean_side?7:6]<=radius_count[mean_side]>=8 && mean_radius>=6 &&
      radius_max[mean_side]-radius_min[mean_side]<=4+(mean_radius>>4);
     if(!mean_side)begin mean_side<=1;state<=10;end else state<=0;
    end
   endcase
  end
 end
function[10:0] normalize_y;input[10:0] d;begin case(d)
11'd0:normalize_y=11'd0;
11'd1:normalize_y=11'd1;
11'd2:normalize_y=11'd2;
11'd3:normalize_y=11'd3;
11'd4:normalize_y=11'd4;
11'd5:normalize_y=11'd5;
11'd6:normalize_y=11'd6;
11'd7:normalize_y=11'd7;
11'd8:normalize_y=11'd8;
11'd9:normalize_y=11'd9;
11'd10:normalize_y=11'd10;
11'd11:normalize_y=11'd11;
11'd12:normalize_y=11'd12;
11'd13:normalize_y=11'd12;
11'd14:normalize_y=11'd13;
11'd15:normalize_y=11'd14;
11'd16:normalize_y=11'd15;
11'd17:normalize_y=11'd16;
11'd18:normalize_y=11'd17;
11'd19:normalize_y=11'd18;
11'd20:normalize_y=11'd19;
11'd21:normalize_y=11'd20;
11'd22:normalize_y=11'd21;
11'd23:normalize_y=11'd22;
11'd24:normalize_y=11'd23;
11'd25:normalize_y=11'd24;
11'd26:normalize_y=11'd25;
11'd27:normalize_y=11'd26;
11'd28:normalize_y=11'd27;
11'd29:normalize_y=11'd28;
11'd30:normalize_y=11'd29;
11'd31:normalize_y=11'd30;
11'd32:normalize_y=11'd31;
11'd33:normalize_y=11'd32;
11'd34:normalize_y=11'd33;
11'd35:normalize_y=11'd34;
11'd36:normalize_y=11'd35;
11'd37:normalize_y=11'd36;
11'd38:normalize_y=11'd36;
11'd39:normalize_y=11'd37;
11'd40:normalize_y=11'd38;
11'd41:normalize_y=11'd39;
11'd42:normalize_y=11'd40;
11'd43:normalize_y=11'd41;
11'd44:normalize_y=11'd42;
11'd45:normalize_y=11'd43;
11'd46:normalize_y=11'd44;
11'd47:normalize_y=11'd45;
11'd48:normalize_y=11'd46;
11'd49:normalize_y=11'd47;
11'd50:normalize_y=11'd48;
11'd51:normalize_y=11'd49;
11'd52:normalize_y=11'd50;
11'd53:normalize_y=11'd51;
11'd54:normalize_y=11'd52;
11'd55:normalize_y=11'd53;
11'd56:normalize_y=11'd54;
11'd57:normalize_y=11'd55;
11'd58:normalize_y=11'd56;
11'd59:normalize_y=11'd57;
11'd60:normalize_y=11'd58;
11'd61:normalize_y=11'd59;
11'd62:normalize_y=11'd60;
11'd63:normalize_y=11'd60;
11'd64:normalize_y=11'd61;
11'd65:normalize_y=11'd62;
11'd66:normalize_y=11'd63;
11'd67:normalize_y=11'd64;
11'd68:normalize_y=11'd65;
11'd69:normalize_y=11'd66;
11'd70:normalize_y=11'd67;
11'd71:normalize_y=11'd68;
11'd72:normalize_y=11'd69;
11'd73:normalize_y=11'd70;
11'd74:normalize_y=11'd71;
11'd75:normalize_y=11'd72;
11'd76:normalize_y=11'd73;
11'd77:normalize_y=11'd74;
11'd78:normalize_y=11'd75;
11'd79:normalize_y=11'd76;
11'd80:normalize_y=11'd77;
11'd81:normalize_y=11'd78;
11'd82:normalize_y=11'd79;
11'd83:normalize_y=11'd80;
11'd84:normalize_y=11'd81;
11'd85:normalize_y=11'd82;
11'd86:normalize_y=11'd83;
11'd87:normalize_y=11'd84;
11'd88:normalize_y=11'd84;
11'd89:normalize_y=11'd85;
11'd90:normalize_y=11'd86;
11'd91:normalize_y=11'd87;
11'd92:normalize_y=11'd88;
11'd93:normalize_y=11'd89;
11'd94:normalize_y=11'd90;
11'd95:normalize_y=11'd91;
11'd96:normalize_y=11'd92;
11'd97:normalize_y=11'd93;
11'd98:normalize_y=11'd94;
11'd99:normalize_y=11'd95;
11'd100:normalize_y=11'd96;
11'd101:normalize_y=11'd97;
11'd102:normalize_y=11'd98;
11'd103:normalize_y=11'd99;
11'd104:normalize_y=11'd100;
11'd105:normalize_y=11'd101;
11'd106:normalize_y=11'd102;
11'd107:normalize_y=11'd103;
11'd108:normalize_y=11'd104;
11'd109:normalize_y=11'd105;
11'd110:normalize_y=11'd106;
11'd111:normalize_y=11'd107;
11'd112:normalize_y=11'd108;
11'd113:normalize_y=11'd108;
11'd114:normalize_y=11'd109;
11'd115:normalize_y=11'd110;
11'd116:normalize_y=11'd111;
11'd117:normalize_y=11'd112;
11'd118:normalize_y=11'd113;
11'd119:normalize_y=11'd114;
11'd120:normalize_y=11'd115;
11'd121:normalize_y=11'd116;
11'd122:normalize_y=11'd117;
11'd123:normalize_y=11'd118;
11'd124:normalize_y=11'd119;
11'd125:normalize_y=11'd120;
11'd126:normalize_y=11'd121;
11'd127:normalize_y=11'd122;
default:normalize_y=d;endcase end endfunction
endmodule
module sqrt22(input clk,rst_n,start,input[21:0] value,output reg done,output reg[10:0] root);
 reg active;reg[3:0] step;reg[21:0] bits;reg[23:0] remn;reg[10:0] work;
 wire[23:0] shifted={remn[21:0],bits[21:20]};
 wire[23:0] trial={11'b0,work,2'b01};
 wire take=shifted>=trial;
 wire[23:0] next_rem=take?shifted-trial:shifted;
 wire[10:0] next_root={work[9:0],take};
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin active<=0;step<=0;bits<=0;remn<=0;work<=0;done<=0;root<=0;end
  else begin
   done<=0;
   if(start && !active)begin active<=1;step<=0;bits<=value;remn<=0;work<=0;end
   else if(active)begin
    remn<=take?shifted-trial:shifted;work<={work[9:0],take};bits<={bits[19:0],2'b0};
    if(step==10)begin active<=0;root<=next_root+(next_rem>next_root && next_root!=2047);done<=1;end else step<=step+1'b1;
   end
  end
 end
endmodule

module mean20(input clk,rst_n,start,input[19:0] numerator,input[7:0] denominator,
 output reg done,output reg[19:0] quotient);
 reg active;reg[4:0] step;reg[19:0] work;reg[8:0] remn;reg[7:0] divisor;
 wire[9:0] shifted={remn,work[19]};wire take=shifted>={2'b0,divisor};
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin active<=0;step<=0;work<=0;remn<=0;divisor<=1;done<=0;quotient<=0;end
  else begin
   done<=0;
   if(start && !active)begin active<=1;step<=0;work<=numerator;remn<=0;divisor<=denominator==0?8'd1:denominator;end
   else if(active)begin
    remn<=take?shifted-divisor:shifted;work<={work[18:0],take};
    if(step==19)begin active<=0;quotient<={work[18:0],take};done<=1;end else step<=step+1'b1;
   end
  end
 end
endmodule
// Calibrated 0.5 mm CAD template; 1 mm boundary tolerance.
// Input is the existing smoothed foreground, before any OSD.
module silhouette_sort(input clk,rst_n,enable,calibrated,vs,de,foreground,rearm,
 input[10:0] x,y,bx,by,bw,bh,input[1:0] measure_status,
 input[19:0] gain_x,gain_y,input[15:0] width10,height10,
 output[2:0] status,output last_good,output servo_out);
 reg[31:0] px_product,py_product;
 reg d0,d1,d2,v0,v1,v2,vprev,fg0,fg1,fg2,range0,range1;
 reg[6:0] column1;
 reg[79:0] inner1,outer1;
 reg expected2,allowed2;
 wire[15:0] cell_x=px_product[31:16],cell_y=py_product[31:16];
 wire coordinate_ok=x>=bx+2 && y>=by+2;
 wire[10:0] dx=x-bx-11'd2,dy=y-by-11'd2;
 wire roi=x>=160 && x<960 && y>=64 && y<548;
 reg[19:0] area,expected_count,missing,extra;
 reg[19:0] area_result,expected_result,missing_result,extra_result;
 reg[4:0] settle;
 reg result_pending,frame_done,seen,reliable_result,good_result;
 reg[26:0] missing_percent,extra_percent,expected_limit,extra_limit;
 reg shape_ok;
 wire dimensions_ok=width10>=370 && width10<=430 && height10>=370 && height10<=430;
 wire period_tick;wire[1:0] target;
 sort_sequence u_sequence(clk,rst_n,enable,calibrated,frame_done,seen,reliable_result,good_result,period_tick,rearm,target,status,last_good);
 servo_pwm drive(clk,rst_n,target,servo_out,period_tick);
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   px_product<=0;py_product<=0;d0<=0;d1<=0;d2<=0;v0<=0;v1<=0;v2<=0;vprev<=0;
   fg0<=0;fg1<=0;fg2<=0;range0<=0;range1<=0;column1<=0;inner1<=0;outer1<=0;
   expected2<=0;allowed2<=0;area<=0;expected_count<=0;missing<=0;extra<=0;
   area_result<=0;expected_result<=0;missing_result<=0;extra_result<=0;
   settle<=0;result_pending<=0;frame_done<=0;seen<=0;reliable_result<=0;good_result<=0;
   missing_percent<=0;extra_percent<=0;expected_limit<=0;extra_limit<=0;shape_ok<=0;
  end else begin
   // Width-limited subtracts and registered products keep the pixel path short.
   px_product<=dx*{gain_x,1'b0};
   py_product<=dy*{gain_y,1'b0};
   d0<=de && roi;v0<=vs;fg0<=foreground;range0<=coordinate_ok && measure_status!=0;
   d1<=d0;v1<=v0;fg1<=fg0;range1<=range0 && cell_x<80 && cell_y<80;
   column1<=cell_x[6:0];inner1<=inner_row(cell_y[6:0]);outer1<=outer_row(cell_y[6:0]);
   d2<=d1;v2<=v1;fg2<=fg1;vprev<=v2;
   expected2<=range1 && inner1[column1];allowed2<=range1 && outer1[column1];
   frame_done<=0;
   if(d2)begin
    if(fg2)area<=area+1'b1;
    if(expected2)expected_count<=expected_count+1'b1;
    if(expected2 && !fg2)missing<=missing+1'b1;
    if(fg2 && !allowed2)extra<=extra+1'b1;
   end
   if(v2 && !vprev)begin
    area_result<=area;expected_result<=expected_count;missing_result<=missing;extra_result<=extra;
    area<=0;expected_count<=0;missing<=0;extra<=0;settle<=20;result_pending<=1;
   end else if(result_pending)begin
    if(settle==18)begin
     missing_percent<=missing_result*100;extra_percent<=extra_result*100;
     expected_limit<=expected_result*3;extra_limit<=area_result*8;
    end
    if(settle==17)shape_ok<=expected_result>=1000 && missing_percent<=expected_limit && extra_percent<=extra_limit;
    if(settle!=0)settle<=settle-1'b1;
    else begin
     // Existing measurement scales settle in ten clocks after the frame boundary.
     // Whole-ROI bright speckles alone are not an actual workpiece.
     seen<=area_result>=1000 && measure_status!=0;
     reliable_result<=measure_status==2;
     good_result<=dimensions_ok && shape_ok;
     frame_done<=1;result_pending<=0;
    end
   end
  end
 end
// CAD mask functions appended by generate_sorting.py.
function[79:0] inner_row;input[6:0] row;begin case(row)
7'd0:inner_row=80'h00000000000000000000;
7'd1:inner_row=80'h00000000000000000000;
7'd2:inner_row=80'h0000fffffc0000000000;
7'd3:inner_row=80'h0000fffffc0000000000;
7'd4:inner_row=80'h0000fffffc0000000000;
7'd5:inner_row=80'h0000fffffc0000000000;
7'd6:inner_row=80'h0001fffffc0000000000;
7'd7:inner_row=80'h0001fffffc0000000000;
7'd8:inner_row=80'h0003fffffc0000000000;
7'd9:inner_row=80'h0003fffffc0000000000;
7'd10:inner_row=80'h0007fffffc0000000000;
7'd11:inner_row=80'h000ffffffc0000000000;
7'd12:inner_row=80'h001ffffffc0000000000;
7'd13:inner_row=80'h003ffffffc0000000000;
7'd14:inner_row=80'h00fffffffc0000000000;
7'd15:inner_row=80'h03fffffffc0000000000;
7'd16:inner_row=80'h3ffffffffc0000000000;
7'd17:inner_row=80'h3ffffffffc0000000000;
7'd18:inner_row=80'h3ffffffffc0000000000;
7'd19:inner_row=80'h3ffffffffc0000000000;
7'd20:inner_row=80'h3ffffffffc0000000000;
7'd21:inner_row=80'h3ffffffffc0000000000;
7'd22:inner_row=80'h3ffffffffc0000000000;
7'd23:inner_row=80'h3ffffffffc0000000000;
7'd24:inner_row=80'h3ffffffffc0000000000;
7'd25:inner_row=80'h3ffffffffc0000000000;
7'd26:inner_row=80'h3ffffffffc0000000000;
7'd27:inner_row=80'h3ffffffffc0000000000;
7'd28:inner_row=80'h07fffffffc0000000000;
7'd29:inner_row=80'h01fffffffc0000000000;
7'd30:inner_row=80'h007ffffffc0000000000;
7'd31:inner_row=80'h003ffffffc0000000000;
7'd32:inner_row=80'h003ffffffc0000000000;
7'd33:inner_row=80'h001ffffffc0000000000;
7'd34:inner_row=80'h001fffffffffffffff80;
7'd35:inner_row=80'h000fffffffffffffffe0;
7'd36:inner_row=80'h000ffffffffffffffff0;
7'd37:inner_row=80'h000ffffffffffffffff8;
7'd38:inner_row=80'h000ffffffffffffffff8;
7'd39:inner_row=80'h000ffffffffffffffffc;
7'd40:inner_row=80'h000ffffffffffffffffc;
7'd41:inner_row=80'h000ffffffffffffffff8;
7'd42:inner_row=80'h000ffffffffffffffff8;
7'd43:inner_row=80'h000ffffffffffffffff0;
7'd44:inner_row=80'h000fffffffffffffffe0;
7'd45:inner_row=80'h001fffffffffffffff80;
7'd46:inner_row=80'h001ffffffc0000000000;
7'd47:inner_row=80'h003ffffffc0000000000;
7'd48:inner_row=80'h003ffffffc0000000000;
7'd49:inner_row=80'h007ffffffc0000000000;
7'd50:inner_row=80'h01fffffffc0000000000;
7'd51:inner_row=80'h07fffffffc0000000000;
7'd52:inner_row=80'h3ffffffffc0000000000;
7'd53:inner_row=80'h3ffffffffc0000000000;
7'd54:inner_row=80'h3ffffffffc0000000000;
7'd55:inner_row=80'h3ffffffffc0000000000;
7'd56:inner_row=80'h3ffffffffc0000000000;
7'd57:inner_row=80'h3ffffffffc0000000000;
7'd58:inner_row=80'h3ffffffffc0000000000;
7'd59:inner_row=80'h3ffffffffc0000000000;
7'd60:inner_row=80'h3ffffffffc0000000000;
7'd61:inner_row=80'h3ffffffffc0000000000;
7'd62:inner_row=80'h3ffffffffc0000000000;
7'd63:inner_row=80'h3ffffffffc0000000000;
7'd64:inner_row=80'h03fffffffc0000000000;
7'd65:inner_row=80'h00fffffffc0000000000;
7'd66:inner_row=80'h003ffffffc0000000000;
7'd67:inner_row=80'h001ffffffc0000000000;
7'd68:inner_row=80'h000ffffffc0000000000;
7'd69:inner_row=80'h0007fffffc0000000000;
7'd70:inner_row=80'h0003fffffc0000000000;
7'd71:inner_row=80'h0003fffffc0000000000;
7'd72:inner_row=80'h0001fffffc0000000000;
7'd73:inner_row=80'h0001fffffc0000000000;
7'd74:inner_row=80'h0000fffffc0000000000;
7'd75:inner_row=80'h0000fffffc0000000000;
7'd76:inner_row=80'h0000fffffc0000000000;
7'd77:inner_row=80'h0000fffffc0000000000;
7'd78:inner_row=80'h00000000000000000000;
7'd79:inner_row=80'h00000000000000000000;
default:inner_row=0;endcase end endfunction
function[79:0] outer_row;input[6:0] row;begin case(row)
7'd0:outer_row=80'h000fffffffc000000000;
7'd1:outer_row=80'h000fffffffc000000000;
7'd2:outer_row=80'h001fffffffc000000000;
7'd3:outer_row=80'h001fffffffc000000000;
7'd4:outer_row=80'h003fffffffc000000000;
7'd5:outer_row=80'h003fffffffc000000000;
7'd6:outer_row=80'h007fffffffc000000000;
7'd7:outer_row=80'h00ffffffffc000000000;
7'd8:outer_row=80'h01ffffffffc000000000;
7'd9:outer_row=80'h03ffffffffc000000000;
7'd10:outer_row=80'h0fffffffffc000000000;
7'd11:outer_row=80'h3fffffffffc000000000;
7'd12:outer_row=80'hffffffffffc000000000;
7'd13:outer_row=80'hffffffffffc000000000;
7'd14:outer_row=80'hffffffffffc000000000;
7'd15:outer_row=80'hffffffffffc000000000;
7'd16:outer_row=80'hffffffffffc000000000;
7'd17:outer_row=80'hffffffffffc000000000;
7'd18:outer_row=80'hffffffffffc000000000;
7'd19:outer_row=80'hffffffffffc000000000;
7'd20:outer_row=80'hffffffffffc000000000;
7'd21:outer_row=80'hffffffffffc000000000;
7'd22:outer_row=80'hffffffffffc000000000;
7'd23:outer_row=80'hffffffffffc000000000;
7'd24:outer_row=80'hffffffffffc000000000;
7'd25:outer_row=80'hffffffffffc000000000;
7'd26:outer_row=80'hffffffffffc000000000;
7'd27:outer_row=80'hffffffffffc000000000;
7'd28:outer_row=80'hffffffffffc000000000;
7'd29:outer_row=80'hffffffffffc000000000;
7'd30:outer_row=80'hfffffffffffffffffff8;
7'd31:outer_row=80'hfffffffffffffffffffe;
7'd32:outer_row=80'h7fffffffffffffffffff;
7'd33:outer_row=80'h1fffffffffffffffffff;
7'd34:outer_row=80'h07ffffffffffffffffff;
7'd35:outer_row=80'h03ffffffffffffffffff;
7'd36:outer_row=80'h03ffffffffffffffffff;
7'd37:outer_row=80'h01ffffffffffffffffff;
7'd38:outer_row=80'h01ffffffffffffffffff;
7'd39:outer_row=80'h00ffffffffffffffffff;
7'd40:outer_row=80'h00ffffffffffffffffff;
7'd41:outer_row=80'h01ffffffffffffffffff;
7'd42:outer_row=80'h01ffffffffffffffffff;
7'd43:outer_row=80'h03ffffffffffffffffff;
7'd44:outer_row=80'h03ffffffffffffffffff;
7'd45:outer_row=80'h07ffffffffffffffffff;
7'd46:outer_row=80'h1fffffffffffffffffff;
7'd47:outer_row=80'h7fffffffffffffffffff;
7'd48:outer_row=80'hfffffffffffffffffffe;
7'd49:outer_row=80'hfffffffffffffffffff8;
7'd50:outer_row=80'hffffffffffc000000000;
7'd51:outer_row=80'hffffffffffc000000000;
7'd52:outer_row=80'hffffffffffc000000000;
7'd53:outer_row=80'hffffffffffc000000000;
7'd54:outer_row=80'hffffffffffc000000000;
7'd55:outer_row=80'hffffffffffc000000000;
7'd56:outer_row=80'hffffffffffc000000000;
7'd57:outer_row=80'hffffffffffc000000000;
7'd58:outer_row=80'hffffffffffc000000000;
7'd59:outer_row=80'hffffffffffc000000000;
7'd60:outer_row=80'hffffffffffc000000000;
7'd61:outer_row=80'hffffffffffc000000000;
7'd62:outer_row=80'hffffffffffc000000000;
7'd63:outer_row=80'hffffffffffc000000000;
7'd64:outer_row=80'hffffffffffc000000000;
7'd65:outer_row=80'hffffffffffc000000000;
7'd66:outer_row=80'hffffffffffc000000000;
7'd67:outer_row=80'hffffffffffc000000000;
7'd68:outer_row=80'h3fffffffffc000000000;
7'd69:outer_row=80'h0fffffffffc000000000;
7'd70:outer_row=80'h03ffffffffc000000000;
7'd71:outer_row=80'h01ffffffffc000000000;
7'd72:outer_row=80'h00ffffffffc000000000;
7'd73:outer_row=80'h007fffffffc000000000;
7'd74:outer_row=80'h003fffffffc000000000;
7'd75:outer_row=80'h003fffffffc000000000;
7'd76:outer_row=80'h001fffffffc000000000;
7'd77:outer_row=80'h001fffffffc000000000;
7'd78:outer_row=80'h000fffffffc000000000;
7'd79:outer_row=80'h000fffffffc000000000;
default:outer_row=0;endcase end endfunction
endmodule
// Detection and clearance durations use the 50 Hz PWM tick, not display FPS.
module sort_sequence #(
 parameter STABLE_FRAMES=8, EMPTY_FRAMES=8,
 parameter INSPECT_PERIODS=50, CLEAR_PERIODS=15, FRESH_PERIODS=5,
 parameter TILT_PERIODS=50, RETURN_PERIODS=50
)(input clk,rst_n,enable,calibrated,frame_done,present,reliable,good,period_tick,rearm,
 output[1:0] target,output reg[2:0] status,output reg last_good);
 reg[1:0] motion; // 0 inspect, 1 tilt, 2 return, 3 wait for departure
 reg[7:0] stable_count,empty_count,periods,inspect_ticks,clear_ticks;
 reg[3:0] frame_age;
 reg previous_good;
 wire fresh=frame_age<FRESH_PERIODS;
 wire valid_target=present && reliable;
 assign target=(enable && calibrated && motion==1)?(last_good?2'd2:2'd1):2'd0;
 always @* begin
  if(!enable)status=7;
  else if(motion==1)status=last_good?3:4;
  else if(motion==2)status=5;
  else if(motion==3)status=6;
  else status=present?2:1;
 end
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   motion<=0;stable_count<=0;empty_count<=0;periods<=0;
   inspect_ticks<=0;clear_ticks<=0;frame_age<=FRESH_PERIODS;previous_good<=0;last_good<=0;
  end else begin
   if(period_tick)begin
    if(frame_age<FRESH_PERIODS)frame_age<=frame_age+1'b1;
    case(motion)
     0:begin
      clear_ticks<=0;
      if(enable && calibrated && fresh && valid_target &&
         stable_count>=STABLE_FRAMES && good==previous_good)begin
       if(inspect_ticks==INSPECT_PERIODS-1)begin
        last_good<=good;motion<=1;periods<=0;inspect_ticks<=0;stable_count<=0;
       end else inspect_ticks<=inspect_ticks+1'b1;
      end else inspect_ticks<=0;
     end
     1:begin
      inspect_ticks<=0;
      if(periods==TILT_PERIODS-1)begin motion<=2;periods<=0;end
      else periods<=periods+1'b1;
     end
     2:begin
      if(periods==RETURN_PERIODS-1)begin motion<=3;periods<=0;clear_ticks<=0;end
      else periods<=periods+1'b1;
     end
     3:begin
      // A settled valid workpiece still blocks repetition. Texture or an
      // invalid/unstable silhouette cannot hold the latch forever.
      if(fresh && !valid_target && empty_count>=EMPTY_FRAMES)begin
       if(clear_ticks==CLEAR_PERIODS-1)begin
        motion<=0;clear_ticks<=0;inspect_ticks<=0;stable_count<=0;empty_count<=0;
       end else clear_ticks<=clear_ticks+1'b1;
      end else clear_ticks<=0;
     end
    endcase
   end
   if(frame_done)begin
    frame_age<=0;
    if(!valid_target)begin
     stable_count<=0;inspect_ticks<=0;
     if(empty_count<EMPTY_FRAMES)empty_count<=empty_count+1'b1;
    end else begin
     empty_count<=0;clear_ticks<=0;
     if(enable && calibrated && motion==0)begin
      previous_good<=good;
      if(previous_good!=good)begin stable_count<=1;inspect_ticks<=0;end
      else if(stable_count<STABLE_FRAMES)stable_count<=stable_count+1'b1;
     end else stable_count<=0;
    end
   end
   if(!enable || !calibrated)begin
    stable_count<=0;inspect_ticks<=0;periods<=0;
    if(motion==1 || motion==2)begin motion<=3;clear_ticks<=0;end
   end
   // Explicit SW2 resume starts a fresh, full-duration inspection. A simple
   // function switch does not discard the already-sorted-object latch.
   if(rearm)begin
    motion<=0;stable_count<=0;empty_count<=0;periods<=0;inspect_ticks<=0;clear_ticks<=0;
   end
  end
 end
endmodule
// User's PM35S specification: 270 degrees over 500..2500 us.
// Absolute 60 / 90 / 120 degree commands. Mount horn level at 90 degrees.
module servo_pwm #(
 parameter CLK_HZ=68493000,
 parameter CENTER_US=1167, BAD_US=944, GOOD_US=1389
)(input clk,rst_n,input[1:0] target,output pwm,output period_tick);
 localparam PERIOD=CLK_HZ/50;
 localparam CENTER_TICKS=(64'd1*CLK_HZ*CENTER_US+500000)/1000000;
 localparam BAD_TICKS=(64'd1*CLK_HZ*BAD_US+500000)/1000000;
 localparam GOOD_TICKS=(64'd1*CLK_HZ*GOOD_US+500000)/1000000;
 reg[20:0] count,high_ticks;
 assign period_tick=count==PERIOD-1;
 assign pwm=rst_n && count<high_ticks;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin count<=0;high_ticks<=CENTER_TICKS;end
  else if(period_tick)begin
   count<=0;
   case(target)1:high_ticks<=BAD_TICKS;2:high_ticks<=GOOD_TICKS;default:high_ticks<=CENTER_TICKS;endcase
  end else count<=count+1'b1;
 end
endmodule
