module mixer(
 input I_clk,I_rst_n,I_key_n,I_function_key_n,
 input I_video_vsync,I_video_hsync,I_video_de,
 output O_vdieo_rd_en,input[23:0] I_video_rd_data,
 output reg O_mixer_vsync,O_mixer_hsync,O_mixer_de,output reg[23:0] O_mixer_data);
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
 wire[1:0] function_id;wire[3:0] algorithm;wire calibrated;wire[7:0] span;
 wire[15:0] width10,height10;wire[10:0] bx,by,bw,bh;wire[1:0] measure_status;
 assign function_id=0;
 assign calibrated=0;assign span=160;assign width10=0;assign height10=0;
 assign bx=0;assign by=0;assign bw=0;assign bh=0;assign measure_status=0;
 reg[1:0] key_sync;reg key_stable;reg[31:0] debounce;
 reg[3:0] requested_algorithm,active_algorithm;reg frame_previous;
 assign algorithm=active_algorithm;
 always @(posedge I_clk or negedge I_rst_n)begin
  if(!I_rst_n)begin key_sync<=3;key_stable<=1;debounce<=0;requested_algorithm<=0;active_algorithm<=0;frame_previous<=0;end
  else begin
   key_sync<={key_sync[0],I_key_n};frame_previous<=vs[1];
   if(key_sync[1]==key_stable)debounce<=0;
   else if(debounce==1369859)begin
    debounce<=0;key_stable<=key_sync[1];
    if(!key_sync[1])requested_algorithm<=requested_algorithm==8?0:requested_algorithm+1'b1;
   end else debounce<=debounce+1'b1;
   if(vs[1] && !frame_previous)active_algorithm<=requested_algorithm;
  end
 end
 wire hole_mode=algorithm==8;
 wire graph_enable=function_id==0 && algorithm==8;
 wire fv,fh,fd,fp,ready,busy;wire[10:0] fx,fy;
 frame_connect connect(.clk(I_clk),.rst_n(I_rst_n),.enable(graph_enable),.refresh(1'b0),.hole_mode(hole_mode),
 .cap_vs(hv),.cap_de(hd),.cap_x(hx),.cap_y(hy),
 .candidate_in(!hole_foreground),
 .seed_in((hx==0 || hx==1023 || hy==0 || hy==599) && !hole_foreground),
 .disp_vs(vs[1]),.disp_hs(hs[1]),.disp_de(de[1]),.disp_x(x),.disp_y(y),
 .ov(fv),.oh(fh),.od(fd),.ox(fx),.oy(fy),.pixel(fp),.ready(ready),.busy(busy));
 reg pv,ph,pd;reg[10:0] px,py;reg[23:0] pixel;
 reg[7:0] numchar;wire[4:0] numrow=py-152;
 wire[15:0] numbits=digit_row({numchar[3:0],numrow});
 always @* begin
  numchar=15;
  case((px-32)>>4)
   0:numchar=10;1:numchar=12;2:numchar=(width10/1000)%10;3:numchar=(width10/100)%10;
   4:numchar=(width10/10)%10;5:numchar=13;6:numchar=width10%10;
   8:numchar=11;9:numchar=12;10:numchar=(height10/1000)%10;11:numchar=(height10/100)%10;
   12:numchar=(height10/10)%10;13:numchar=13;14:numchar=height10%10;16:numchar=14;17:numchar=14;
  endcase
 end
 wire[575:0] title_bits=text_row(py-32);
 wire[93:0] red_bits=red_row(py-16),blue_bits=blue_row(py-16);
 wire[239:0] mode_bits=algorithm==0?label_0(py-32):algorithm==1?label_1(py-32):algorithm==2?label_2(py-32):
 algorithm==3?label_3(py-32):algorithm==4?label_4(py-32):algorithm==5?label_5(py-32):algorithm==6?label_6(py-32):
 algorithm==7?label_7(py-32):label_8(py-32);
 wire[699:0] feature_bits=function_id==0?vision_row(py-76):function_id==1?measure_row(py-76):sorting_row(py-76);
 wire[699:0] state_bits=!calibrated?cal_row(py-112):measure_status==0?waiting_row(py-112):measure_status==1?stable_row(py-112):valid_row(py-112);
 wire[699:0] busy_bits=processing_row(py-112);
 wire[899:0] guide_bits=guide_row(py-560);
 always @* begin
  pv=vs[1];ph=hs[1];pd=de[1];px=x;py=y;pixel=rgb;
  if(function_id==0)case(algorithm)
   1:pixel={gray,gray,gray};
   2:begin pv=gv;ph=gh;pd=gd;px=gx;py=gy;pixel={gaussian,gaussian,gaussian};end
   3:pixel=gray>=128?24'hffffff:24'h0;
   4,5:begin pv=mv;ph=mh;pd=md;px=mx;py=my;pixel=(algorithm==4?opened:closed)?24'hffffff:24'h0;end
   6:begin pv=ev;ph=eh;pd=ed;px=ex;py=ey;pixel=sobel_edge?24'hffffff:24'h0;end
   8:begin pv=fv;ph=fh;pd=fd;px=fx;py=fy;pixel=fp?24'hffffff:24'h0;end
   7:begin pv=cv;ph=ch;pd=cd;px=cx;py=cy;pixel=contour?24'hffffff:24'h0;end
   default:pixel=rgb;
  endcase
  if(px>=16 && px<110 && py>=16 && py<103)begin
   if(red_bits[px-16])pixel=24'hff2028;if(blue_bits[px-16])pixel=24'h1c2c50;
  end
  if(px>=128 && px<704 && py>=32 && py<72)if(title_bits[px-128])pixel=24'hffffff;
  if(function_id==0 && px>=768 && px<1008 && py>=32 && py<64)if(mode_bits[px-768])pixel=24'h20ff60;
  if(px>=128 && px<828 && py>=76 && py<108)if(feature_bits[px-128])pixel=24'h20ff60;
  if(graph_enable && busy && px>=32 && px<732 && py>=112 && py<144)if(busy_bits[px-32])pixel=24'hffff20;
  if(function_id==1)begin
   if(px>=32 && px<732 && py>=112 && py<144)if(state_bits[px-32])pixel=24'hffff20;
   if(calibrated && measure_status==2 && px>=32 && px<320 && py>=152 && py<184)if(numbits[(px-32)&15])pixel=24'hffff20;
   if(!calibrated && py>=450 && py<555 && (px==256 || px==256+span))pixel=24'hff2020;
   if(calibrated && measure_status!=0 && ((px>=bx && px<bx+bw && (py==by || py==by+bh-1)) || (py>=by && py<by+bh && (px==bx || px==bx+bw-1))))pixel=24'h20ff60;
   if((px==160 || px==863) && py>=12 && py<440)pixel=24'hffcc00;
   if((py==12 || py==439) && px>=160 && px<864)pixel=24'hffcc00;
  end
  if(px>=32 && px<932 && py>=560 && py<592)if(guide_bits[px-32])pixel=24'hffff20;
 end
 always @(posedge I_clk or negedge I_rst_n)begin
  if(!I_rst_n)begin O_mixer_vsync<=0;O_mixer_hsync<=0;O_mixer_de<=0;O_mixer_data<=0;end
  else begin O_mixer_vsync<=pv;O_mixer_hsync<=ph;O_mixer_de<=pd;O_mixer_data<=pd?pixel:24'b0;end
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
function [239:0] label_8; input[4:0] row; begin case(row)
5'd6: label_8=240'h38706000000001c00000000;
5'd7: label_8=240'h3870f3fe000001c0000e01c;
5'd8: label_8=240'h3870f3fe000001c0000e01c;
5'd9: label_8=240'h387063fe000001c0000e01c;
5'd10: label_8=240'h3870000e000001c0000e01c;
5'd11: label_8=240'h3870000e000001c0000e01c;
5'd12: label_8=240'h3870700e000f81c0fc0e01c;
5'd13: label_8=240'h3870700e003fc1c1fe0e01c;
5'd14: label_8=240'h387073fe003fe1c3ff0fffc;
5'd15: label_8=240'h387073fe0078f1c7878fffc;
5'd16: label_8=240'h387073fe007071c7038fffc;
5'd17: label_8=240'h3870700e007ff1c7038e01c;
5'd18: label_8=240'h3870700e007ff1c7038e01c;
5'd19: label_8=240'h3870700e000071c7038e01c;
5'd20: label_8=240'h3870700e0020f1c7878e01c;
5'd21: label_8=240'h3870700e003fe1c3ff0e01c;
5'd22: label_8=240'h3870700e003fc1c1fe0e01c;
5'd23: label_8=240'h3870700e001f81c0fc0e01c;
default: label_8=0; endcase end endfunction
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
5'd6: measure_row=700'h700004078000000000;
5'd7: measure_row=700'h1ffff8707fde0780001ffff8;
5'd8: measure_row=700'h1ffff8767ffe0780001ffff8;
5'd9: measure_row=700'h1c0038767ffc0780001ffff8;
5'd10: measure_row=700'h1ffff87661d87ffffe1c0078;
5'd11: measure_row=700'h1c00387661d07ffffe1c0078;
5'd12: measure_row=700'h1ffff8766dc47ffffe1c0078;
5'd13: measure_row=700'h1ffff8766dce0780001c0078;
5'd14: measure_row=700'h766dde0780201c0078;
5'd15: measure_row=700'hfffffe766dfc0780781ffff8;
5'd16: measure_row=700'hfffffe766dd80780f81ffff8;
5'd17: measure_row=700'h766dc80781f01ffff8;
5'd18: measure_row=700'h1ffff8766dc00783e01c1c78;
5'd19: measure_row=700'h1c3c38766dc80787c0001c78;
5'd20: measure_row=700'h1ffff8766df8078380003c78;
5'd21: measure_row=700'h1c3c38766ff8078100003838;
5'd22: measure_row=700'h1ffff8760e1c078000007838;
5'd23: measure_row=700'h3c0076161c07800000f03c;
5'd24: measure_row=700'h3ffffc763f1c07800001f03c;
5'd25: measure_row=700'h3ffffc703b9c03800007e03c;
5'd26: measure_row=700'h3c007073de03ff001fc01e;
5'd27: measure_row=700'hffffff7ee1ee03fe007f801f;
5'd28: measure_row=700'hffffff3ee1ce01fe003e000c;
5'd29: measure_row=700'h3e408c000000180004;
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
5'd6: cal_row=700'h1e000000000018003c000006703000003000000000000000180000003801c020;
5'd7: cal_row=700'h1e001ffff8003c003c00000e70700000f07ffffe000000003c007ffe3801c070;
5'd8: cal_row=700'h1f003f0001e001ffff8003c003c07fe1e70787ffc707ffffe000000003c007ffe3801c0f0;
5'd9: cal_row=700'h7f80ffc001e001ffff87ffffe3c07fe1c70387ffc787ffffe0000007ffffe7ffe387ffe78;
5'd10: cal_row=700'hffc0ffc001e001c00787ffffefffffe78703c7ffc3801c3800000007ffffe0000ff7ffe3c;
5'd11: cal_row=700'hf1e1f0c001e001c00787ffffeffff007fff9c0381bc01c3800000007ffffe0000ff7ffe1e;
5'd12: cal_row=700'hf1e1e00001e001c00780781e0ffff8c7fffde03839c01c38000000078001e0000ff01c00e;
5'd13: cal_row=700'h3c3cf03c3cf1e0f1e003ffe001c007803c3e03c038e00ffce0383ce3ffffc00006078001e7fff3801c00c;
5'd14: cal_row=700'h7f7ff07f7ff1e0f1e003ffe001c007801e7c03c139e00e0ff0381fe3ffffc0000f00000007fff3cffff64;
5'd15: cal_row=700'hfffff0fffff1e0f1f003ffe001ffff800ff803c3bbc70e0fe0381ff3ffffc0000f07ffffe7fff3cffffe0;
5'd16: cal_row=700'hf9f9f0f9f9f1e0f0f80001e001ffff8007e003c7bf87ff07e0380fe39c39c0000607ffffe01c07cffff70;
5'd17: cal_row=700'hf0f0f0f0f0f1e0f07c0001e001ffff807fff03c71f87ffe7003807238e39c0000007ffffe01c1fe000078;
5'd18: cal_row=700'hf0f0f0f0f0f1e0f03e0001e001c1c787ffffe3cf1f01ffe3803807838e79c00000000380019ccfe1e0038;
5'd19: cal_row=700'hf0f0f0f0f0f1e0f01f0001e00001c787f81fe3de1f038fe3c03803c39efdc0000000038701ddcbfffff3c;
5'd20: cal_row=700'hf0f0f0f0f0f1e0f0078001e00003c783f81fe3dc1e03ce1fe0381fe3befdc0000003ff87039ce3fffff3e;
5'd21: cal_row=700'hf0f0f0f0f0f0f1f003c001e000038380381c03c43f01fe0fe0381fe3ff5fc0000003ff87039ce3affff3e;
5'd22: cal_row=700'hf0f0f0f0f0f0f1e003e001e000078380381c03c03f80fc07e0381fe3f71fc0000603ff87879ce3a1e0c3e;
5'd23: cal_row=700'hf0f0f0f0f0f07fe1ffe001e0000f03c0381e03c07f827c00603800c3e78fc0000f000387871c7381e1e3a;
5'd24: cal_row=700'hf0f0f0f0f0f07fc1ffe001e0001f03c0381e03c0fbce3f0000380003a3cfc0000f00038fc71c7381e3c38;
5'd25: cal_row=700'hf0f0f0f0f0f01f01ffe001e0007e03c0380f03c071e73fff00381f0383c7c0000600039fcf1c7b81e3838;
5'd26: cal_row=700'h7ffffe1fc01e0380f81fe20f77ffff7ffffe38103c0000007ffffe21c2381e1038;
5'd27: cal_row=700'h7ffffe7f801f03807c1fe00e7f1ffe7ffffe3f001c0000007fffce01f8381fe038;
5'd28: cal_row=700'h7ffffe3e000c0380380fc0043f06067ffe0e3f001c0000007ffe0e01f8380fe038;
5'd29: cal_row=700'h1800040380100000001c00000000001f001c00000000000400f83807e038;
default: cal_row=0; endcase end endfunction
function [699:0] waiting_row; input[4:0] row; begin case(row)
5'd6: waiting_row=700'h3870000403c038003c0000007000003800000008e00003806001008001c020038070;
5'd7: waiting_row=700'h7ffe38707fde03e038003c007ff8707ffe381ffff81ee000038cf00781e001c07003c078;
5'd8: waiting_row=700'h7ffe38767ffe07e0383c3c3c3ff8707ffe381ffff81ce000039c7007c3c001c0f07ffffc;
5'd9: waiting_row=700'h7ffe38767ffc0ff0383c3c3c3ffbfe7ffe381ffff8ffffff039e7003c3c07ffe787ffffc;
5'd10: waiting_row=700'hefe7661d80e79fe3c3c3c39c3fe0000ff1c0038ffffff038e783ffffc7ffe3c0ef1ce;
5'd11: waiting_row=700'hefe7661d01e3dfe3c3c3c38e3fe0000ff1c0038ffffff7ffe383ffffc7ffe1e1c7dce;
5'd12: waiting_row=700'h7ffefe766dc47c1ffe3c3c3c38e0700000ff1c003800e0007ffe383ffffc01c00e0c7c84;
5'd13: waiting_row=700'h7ffe38766dceffff383c3c3c38e0707fff381ffff818e0007fff3c383c1c01c00c003c00;
5'd14: waiting_row=700'h7ffe38766dde7fff383ffffc3f70707fff3c1ffff878e7fc03873c383c1cffff643ffffc;
5'd15: waiting_row=700'h70e3c766dfc2ffa7c3ffffc3f3fff7fff3c1ffff838e7fc03873e3ffffcffffe03ffffc;
5'd16: waiting_row=700'h70efc766dd80000fc3ffffc1e17ff01c07c1c00383ce7fc0387be3ffffcffff703ffffc;
5'd17: waiting_row=700'h3feefe766dc818c1fe003c000007ff01c1fe1c00381de71c03823f3ffffc000078003c00;
5'd18: waiting_row=700'h3feefe766dc038c1be003c003ff1c019ccfe1c00380fe71c03803e383c1c1e0038ffffff;
5'd19: waiting_row=700'h70e7f766dc839cebf383c1c3ff1ce1ddcbf1ffff80fc71cffffbe383c1cffff3cffffff;
5'd20: waiting_row=700'h70e3f766df839ce3f383c1c3871ce39ce3f1ffff807c7fcffffba3ffffcffff3effffff;
5'd21: waiting_row=700'h7ffe3a766ff81dcc3a383c1c3877ce39ce3a1ffff803c7fcffffba3ffffcffff3e078000;
5'd22: waiting_row=700'h7ffe3a760e1c1ddc3a383c1c3877ce79ce3a1c003823c7fc0380383ffffc1e0c3e7ffffe;
5'd23: waiting_row=700'h7ffe3876161c1d9c38383c1c3ff7ce71c7381c0038e3c000038038003c001e1e3a7ffffe;
5'd24: waiting_row=700'he38763f1c0c0838383c1c3ff1de71c7381c003873e000038038ffffff1e3c387ffffe;
5'd25: waiting_row=700'he38703b9c0e0038383c1c0001fef1c7b81ffff873f7c0038038ffffff1e38380781e0;
5'd26: waiting_row=700'h7ffe387073de7fff383ffffc7ffffe21c2381ffff8773fff038038ffffff1e10380783c0;
5'd27: waiting_row=700'h7ffe387ee1ee7fff383ffffc7fffff01f8381ffff87f1ffe038038003c001fe03807fbc0;
5'd28: waiting_row=700'h7ffe383ee1ce7fff383ffffc7fffc601f8381c00383e087e038038003c000fe03803f880;
5'd29: waiting_row=700'h383e408c00003838000000000400f8381c00381c0000038038003c0007e03803f800;
default: waiting_row=0; endcase end endfunction
function [699:0] stable_row; input[4:0] row; begin case(row)
5'd6: stable_row=700'h700001c07003806000000001c03800003001e008000000003c00000000700004;
5'd7: stable_row=700'h700000e070038cf03ffffc01c0383fff7001e01e000000003c001ffff8707fde;
5'd8: stable_row=700'h70001ff7fe039c703ffffc01c0383fff707fff9c000000003c001ffff8767ffe;
5'd9: stable_row=700'h70001ff7fe039e703ffffc3ffe383fff787fffbc000000003c001c0038767ffc;
5'd10: stable_row=700'h70000e3870038e78003c003ffefe38077801e038000000003c001ffff87661d8;
5'd11: stable_row=700'h70700f3ffe7ffe38003c003ffefe3807383fff100000007ffffe1c00387661d0;
5'd12: stable_row=700'h70703ff3fe7ffe38003c0001c0fe38073c3fff000000007ffffe1ffff8766dc4;
5'd13: stable_row=700'h3ff0703ff0707fff3c003c0001c0383fff3c01e0000000607ffffe1ffff8766dce;
5'd14: stable_row=700'h3ff0703ff7ff03873c003c00ffff383fff3cffffc00000f0783c1e000000766dde;
5'd15: stable_row=700'h3ff0703387ff03873e003c00ffff383fff3effffdf0000f0783c1efffffe766dfc;
5'd16: stable_row=700'h70703380000387be003c00ffff3801e03e00001f000060783c1efffffe766dd8;
5'd17: stable_row=700'h7070fffbfe03823f003c001e00f801e03f3fff1f000000783c1e000000766dc8;
5'd18: stable_row=700'h7070fffbfe03803e003c001e00ff7fffbe3fff1c000000783c1e1ffff8766dc0;
5'd19: stable_row=700'h7070fffb0effffbe003c007fffff7fffbe38071c0000007ffffe1c3c38766dc8;
5'd20: stable_row=700'h70703383feffffba003c007fff3e7fffba3fff1c0000007ffffe1ffff8766df8;
5'd21: stable_row=700'h70703383feffffba003c007fff3803f8383fff1c0000007ffffe1c3c38766ff8;
5'd22: stable_row=700'h70703ff30e038038003c001e1c3807fc3838071c000060783c1e1ffff8760e1c;
5'd23: stable_row=700'h70703ff30e038038003c001e1c381ffe3838075c0000f0783c1e003c0076161c;
5'd24: stable_row=700'h70703ff3fe038038003c001e3c383defb83fff7c0000f0003c003ffffc763f1c;
5'd25: stable_row=700'h70703383fe038038003c001e3838f9e7f83fff7c000060003c003ffffc703b9c;
5'd26: stable_row=700'h7ffffe03830e0380387ffffe1e103871e3b838077c000000003c00003c007073de;
5'd27: stable_row=700'h7ffffe03f30e0380387ffffe0fe03e21e0b83c073c000000003c00ffffff7ee1ee;
5'd28: stable_row=700'h7ffffe03f3ee0380387ffffe0fe03e01e0383fc71c000000003c00ffffff3ee1ce;
5'd29: stable_row=700'h1f3ee03803800000007e01e01e0381fc708000000003c000000003e408c;
default: stable_row=0; endcase end endfunction
function [699:0] valid_row; input[4:0] row; begin case(row)
5'd6: valid_row=700'h700004078000000000;
5'd7: valid_row=700'h1ffff8707fde0780001ffff8;
5'd8: valid_row=700'he0000000000007000001ffff8767ffe0780001ffff8;
5'd9: valid_row=700'h1c0000000000003800001c0038767ffc0780001ffff8;
5'd10: valid_row=700'h380000000000001c00001ffff87661d87ffffe1c0078;
5'd11: valid_row=700'h380000000000000e00001c00387661d07ffffe1c0078;
5'd12: valid_row=700'h700000000000000e00001ffff8766dc47ffffe1c0078;
5'd13: valid_row=700'h700f0f3c0f0f3c0700001ffff8766dce0780001c0078;
5'd14: valid_row=700'h701fdffc1fdffc070000000000766dde0780201c0078;
5'd15: valid_row=700'he03ffffc3ffffc038000fffffe766dfc0780781ffff8;
5'd16: valid_row=700'he03e7e7c3e7e7c038000fffffe766dd80780f81ffff8;
5'd17: valid_row=700'he03c3c3c3c3c3c038000000000766dc80781f01ffff8;
5'd18: valid_row=700'he03c3c3c3c3c3c0380001ffff8766dc00783e01c1c78;
5'd19: valid_row=700'he03c3c3c3c3c3c0380001c3c38766dc80787c0001c78;
5'd20: valid_row=700'he03c3c3c3c3c3c0380001ffff8766df8078380003c78;
5'd21: valid_row=700'he03c3c3c3c3c3c0380001c3c38766ff8078100003838;
5'd22: valid_row=700'he03c3c3c3c3c3c0780001ffff8760e1c078000007838;
5'd23: valid_row=700'h703c3c3c3c3c3c070000003c0076161c07800000f03c;
5'd24: valid_row=700'h703c3c3c3c3c3c0700003ffffc763f1c07800001f03c;
5'd25: valid_row=700'h703c3c3c3c3c3c0e00003ffffc703b9c03800007e03c;
5'd26: valid_row=700'h380000000000000e0000003c007073de03ff001fc01e;
5'd27: valid_row=700'h380000000000001c0000ffffff7ee1ee03fe007f801f;
5'd28: valid_row=700'h1c000000000000380000ffffff3ee1ce01fe003e000c;
5'd29: valid_row=700'he0000000000007000000000003e408c000000180004;
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
5'd6: guide_row=900'h180038000000007800000700000e1c00000000000000780200701c00e078000000381c00001c00000000000000;
5'd7: guide_row=900'h1f80381fffff80780000078003ff0c1fffff80000000780f80701e00e07f8000003c1c3fff1c00000000000000;
5'd8: guide_row=900'h1ff3ff9fffff8078ff80078003ff8e1fffff80000000781f1ffbff00e07f81f00ffe1c3fff1c03f0f03e03c7e0;
5'd9: guide_row=900'hff3ff9fffff8078ff8ffffe03c1ce1fffff8000001fffce1fffff80e01f87fc0ffe1c3fff1c0ffcf03e03cff8;
5'd10: guide_row=900'h1f3ff800f001ffeff8ffffe01c1ee1e03878000001fffc8039e779fff1c0ffe0fff1c1c381c0ffcf03e078ffc;
5'd11: guide_row=900'h70860ffffe1ffe000ffffe1fffff1effc78000001fffc0038c739fff1c0f1e0f07ff1c381c1f0c787e07883e;
5'd12: guide_row=900'h71c70ffffe1ffe000f000e1fffef1efff780000000780007ffff9fffff8e0f0787ff1c39fc1e00787f07801e;
5'd13: guide_row=900'h71ee0ffffe1e38000f000e1c30ef1ef0ff80001800780107fffe1ce7ff9e0f1fffff1c39ff9e00787f07801e;
5'd14: guide_row=900'h3ff7ff8f0f0e1e39ffcffffe1c30ef9e79df80003c00780787000e1ce7ff9e0f1fff1c1c39ff9e00387f0f003e;
5'd15: guide_row=900'h3ff7ff8f0f0e1e39ffcffffe1fffef9e3f9780003c00780f87fffe1ce71c1e0f1fff1c1c381f1f003c730f007c;
5'd16: guide_row=900'h3ff7ff8ffffe1e39ffcffffe1fffef9e3fc780001800780607fffe1ce71e1f1f1ce37c1c381c0f803cf38f03f8;
5'd17: guide_row=900'he70380ffffe1e381e0f000e041f0fdfffff8000003fffe407000e1ce73e1ffe1ce37c1c381c07c03cf38e07f0;
5'd18: guide_row=900'he73ff8f0f0e0e380e0f000e0e3fcf9fe0ff8000003fffe007fffe1ce77f1ffc1ce37f9c381c03e01cf18e0fc0;
5'd19: guide_row=900'he73ff8f0f0e0e384e0f000e0fbbfe9e1fc78000003fffe407fffe1fffff1ef81ce33f9c381c01f01ee19e1f00;
5'd20: guide_row=900'he73ff8ffffe0e38ef0ffffe07fcfe9e7fc7800000001e1e07000e1fffff8e007fffdf9c3c1c00781fe1de1e00;
5'd21: guide_row=900'he78380ffffe0e38e70ffffe07f72e1e7807800000011e0e07fffe1fff5f8f007fffdc1c3c9c003c0fe1dc1e02;
5'd22: guide_row=900'he393b0ffffe0e3dc70ffffe077bce1e07f780000003cf0e07fffe1ce71dcf827fffdc1c3cdc003e0fc0fc1f06;
5'd23: guide_row=900'he3bbb0f0f180e3dc78f000e0efdfe1efff780003807878f00e07000e01d87fe01f01c1c1cfc1ffe0fc0fc0ffe;
5'd24: guide_row=900'he3bbb80077c0e1fff8f000e0eeeee1eff878000380787cf3fffffc0e01c83fe01f81c1c1efc1ffe0fc0fc0ffe;
5'd25: guide_row=900'he3d3b8007f80e1fffcf000e1ee7ae1e600780001c0f83c73fffffc0e01c80fe07bc1c1e1f7e1ffe07c0f803fc;
5'd26: guide_row=900'he1c39c03ff00fefff8ffffe3ce3ee1fffff80001c1fffe700e03c00e01c00000f1f1c1fef1c00000000000000;
5'd27: guide_row=900'he1e3f1fffff8fcf010ffffe187ffe1fffff80000e1fffe780e03e00e01c00003e0fdf0fe7c800000000000000;
5'd28: guide_row=900'he1e3e1ffeff87c7000ffffe087efe1fffff80000e1c7fc380e01e00e01c00003c07df07c78000000000000000;
5'd29: guide_row=900'he081e0fc00f0006000f000e003c2e1e00078000000c000200e00c00e01c000010018f00020000000000000000;
default: guide_row=0; endcase end endfunction
function [15:0] digit_row; input[8:0] addr; begin case(addr)
9'd3: digit_row=16'hf0;
9'd4: digit_row=16'h3f8;
9'd5: digit_row=16'h30c;
9'd6: digit_row=16'h70c;
9'd7: digit_row=16'h706;
9'd8: digit_row=16'h7c6;
9'd9: digit_row=16'h6e6;
9'd10: digit_row=16'h676;
9'd11: digit_row=16'h61e;
9'd12: digit_row=16'h60e;
9'd13: digit_row=16'h70e;
9'd14: digit_row=16'h30c;
9'd15: digit_row=16'h1fc;
9'd16: digit_row=16'hf0;
9'd35: digit_row=16'h70;
9'd36: digit_row=16'h7c;
9'd37: digit_row=16'h6e;
9'd38: digit_row=16'h64;
9'd39: digit_row=16'h60;
9'd40: digit_row=16'h60;
9'd41: digit_row=16'h60;
9'd42: digit_row=16'h60;
9'd43: digit_row=16'h60;
9'd44: digit_row=16'h60;
9'd45: digit_row=16'h60;
9'd46: digit_row=16'h60;
9'd47: digit_row=16'h3fe;
9'd48: digit_row=16'h3fe;
9'd67: digit_row=16'hf0;
9'd68: digit_row=16'h1fc;
9'd69: digit_row=16'h388;
9'd70: digit_row=16'h300;
9'd71: digit_row=16'h300;
9'd72: digit_row=16'h300;
9'd73: digit_row=16'h180;
9'd74: digit_row=16'h180;
9'd75: digit_row=16'hc0;
9'd76: digit_row=16'h60;
9'd77: digit_row=16'h30;
9'd78: digit_row=16'h18;
9'd79: digit_row=16'h7fc;
9'd80: digit_row=16'h7fc;
9'd99: digit_row=16'hf8;
9'd100: digit_row=16'h1fc;
9'd101: digit_row=16'h384;
9'd102: digit_row=16'h300;
9'd103: digit_row=16'h300;
9'd104: digit_row=16'h180;
9'd105: digit_row=16'hf0;
9'd106: digit_row=16'h3f0;
9'd107: digit_row=16'h700;
9'd108: digit_row=16'h600;
9'd109: digit_row=16'h600;
9'd110: digit_row=16'h700;
9'd111: digit_row=16'h3fc;
9'd112: digit_row=16'hfc;
9'd131: digit_row=16'h1c0;
9'd132: digit_row=16'h1e0;
9'd133: digit_row=16'h1e0;
9'd134: digit_row=16'h1b0;
9'd135: digit_row=16'h198;
9'd136: digit_row=16'h18c;
9'd137: digit_row=16'h18c;
9'd138: digit_row=16'h186;
9'd139: digit_row=16'h183;
9'd140: digit_row=16'h7ff;
9'd141: digit_row=16'h7ff;
9'd142: digit_row=16'h180;
9'd143: digit_row=16'h180;
9'd144: digit_row=16'h180;
9'd163: digit_row=16'h3fc;
9'd164: digit_row=16'h3fc;
9'd165: digit_row=16'hc;
9'd166: digit_row=16'hc;
9'd167: digit_row=16'hc;
9'd168: digit_row=16'hc;
9'd169: digit_row=16'h1fc;
9'd170: digit_row=16'h3fc;
9'd171: digit_row=16'h700;
9'd172: digit_row=16'h600;
9'd173: digit_row=16'h600;
9'd174: digit_row=16'h300;
9'd175: digit_row=16'h3fc;
9'd176: digit_row=16'hfc;
9'd195: digit_row=16'h3e0;
9'd196: digit_row=16'h3f8;
9'd197: digit_row=16'h1c;
9'd198: digit_row=16'hc;
9'd199: digit_row=16'h6;
9'd200: digit_row=16'h1f6;
9'd201: digit_row=16'h3fe;
9'd202: digit_row=16'h70e;
9'd203: digit_row=16'h606;
9'd204: digit_row=16'h606;
9'd205: digit_row=16'h606;
9'd206: digit_row=16'h30c;
9'd207: digit_row=16'h3fc;
9'd208: digit_row=16'hf0;
9'd227: digit_row=16'h7fe;
9'd228: digit_row=16'h7fe;
9'd229: digit_row=16'h600;
9'd230: digit_row=16'h300;
9'd231: digit_row=16'h300;
9'd232: digit_row=16'h180;
9'd233: digit_row=16'h180;
9'd234: digit_row=16'hc0;
9'd235: digit_row=16'hc0;
9'd236: digit_row=16'h60;
9'd237: digit_row=16'h60;
9'd238: digit_row=16'h30;
9'd239: digit_row=16'h38;
9'd240: digit_row=16'h18;
9'd259: digit_row=16'h1f8;
9'd260: digit_row=16'h3fc;
9'd261: digit_row=16'h606;
9'd262: digit_row=16'h606;
9'd263: digit_row=16'h60e;
9'd264: digit_row=16'h39c;
9'd265: digit_row=16'h1f8;
9'd266: digit_row=16'h1f8;
9'd267: digit_row=16'h39c;
9'd268: digit_row=16'h606;
9'd269: digit_row=16'h606;
9'd270: digit_row=16'h70e;
9'd271: digit_row=16'h3fc;
9'd272: digit_row=16'h1f8;
9'd291: digit_row=16'hf0;
9'd292: digit_row=16'h3fc;
9'd293: digit_row=16'h30c;
9'd294: digit_row=16'h606;
9'd295: digit_row=16'h606;
9'd296: digit_row=16'h606;
9'd297: digit_row=16'h70e;
9'd298: digit_row=16'h7fc;
9'd299: digit_row=16'h6f8;
9'd300: digit_row=16'h600;
9'd301: digit_row=16'h300;
9'd302: digit_row=16'h380;
9'd303: digit_row=16'h1fc;
9'd304: digit_row=16'h7c;
9'd323: digit_row=16'hc06;
9'd324: digit_row=16'hc06;
9'd325: digit_row=16'hc06;
9'd326: digit_row=16'hc06;
9'd327: digit_row=16'hc06;
9'd328: digit_row=16'hc66;
9'd329: digit_row=16'hc66;
9'd330: digit_row=16'h4ec;
9'd331: digit_row=16'h4ec;
9'd332: digit_row=16'h6ac;
9'd333: digit_row=16'h7bc;
9'd334: digit_row=16'h79c;
9'd335: digit_row=16'h71c;
9'd336: digit_row=16'h71c;
9'd355: digit_row=16'h606;
9'd356: digit_row=16'h606;
9'd357: digit_row=16'h606;
9'd358: digit_row=16'h606;
9'd359: digit_row=16'h606;
9'd360: digit_row=16'h606;
9'd361: digit_row=16'h7fe;
9'd362: digit_row=16'h7fe;
9'd363: digit_row=16'h606;
9'd364: digit_row=16'h606;
9'd365: digit_row=16'h606;
9'd366: digit_row=16'h606;
9'd367: digit_row=16'h606;
9'd368: digit_row=16'h606;
9'd390: digit_row=16'h70;
9'd391: digit_row=16'h70;
9'd392: digit_row=16'h70;
9'd398: digit_row=16'h70;
9'd399: digit_row=16'h70;
9'd400: digit_row=16'h70;
9'd430: digit_row=16'h60;
9'd431: digit_row=16'hf0;
9'd432: digit_row=16'h60;
9'd454: digit_row=16'h336;
9'd455: digit_row=16'h7fe;
9'd456: digit_row=16'h6ee;
9'd457: digit_row=16'h666;
9'd458: digit_row=16'h666;
9'd459: digit_row=16'h666;
9'd460: digit_row=16'h666;
9'd461: digit_row=16'h666;
9'd462: digit_row=16'h666;
9'd463: digit_row=16'h666;
9'd464: digit_row=16'h666;
default:digit_row=0;endcase end endfunction
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
 reg rv,rh,rd;reg[10:0] rx,ry,gradient;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin rv<=0;rh<=0;rd<=0;rx<=0;ry<=0;gradient<=0;end
  else begin rv<=sv;rh<=sh;rd<=sd;rx<=sx;ry<=sy;
   gradient<=sx>=4 && sy>=4 ? magnitude : 0;
  end
 end
 assign ev=rv;assign eh=rh;assign ed=rd;assign ex=rx;assign ey=ry;
 // Gaussian suppresses texture noise; no bright-side gate that loses dark edges.
 assign sobel_edge=gradient>=96;
 assign measurement_edge=sobel_edge;
 // Unused legacy ports, retained for module interface compatibility only.
 assign nv=rv;assign nh=rh;assign nd=rd;assign nx=rx;assign ny=ry;
 assign weak_edge=sobel_edge;assign strong_edge=sobel_edge;
 hole_preprocess #(W) hp(clk,rst_n,gv,gh,gd,gx,gy,gaussian,
  hv,hh,hd,hx,hy,hole_foreground);
endmodule
// Gaussian input; frame-stable contrast threshold, majority denoise, closing.
// Bright objects are foreground. Keep an explicit background frame for flooding.
module hole_preprocess #(parameter W=1024,H=600)(
 input clk,rst_n,vs,hs,de,input[10:0] x,y,input[7:0] gray,
 output ov,oh,od,output[10:0] ox,oy,output foreground);
 reg prev_vs,seen;reg[7:0] minimum,maximum,threshold;
 wire[8:0] midpoint={1'b0,minimum}+{1'b0,maximum};
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin prev_vs<=0;seen<=0;minimum<=255;maximum<=0;threshold<=128;end
  else begin
   prev_vs<=vs;
   if(vs && !prev_vs)begin
    if(seen && maximum-minimum>=24)threshold<=midpoint[8:1];
    minimum<=255;maximum<=0;seen<=0;
   end else if(de && x>=2 && y>=2 && x<W && y<H)begin
    seen<=1;if(gray<minimum)minimum<=gray;if(gray>maximum)maximum<=gray;
   end
  end
 end
 wire av,ah,ad,bv,bh,bd;wire[10:0] ax,ay,bx,by;
 wire[8:0] a,b,c;
 wire binary_pixel=x>=2 && y>=2 && gray>=threshold;
 window3 #(W,1) majority_window(clk,rst_n,vs,hs,de,x,y,binary_pixel,
 av,ah,ad,ax,ay,a[0],a[1],a[2],a[3],a[4],a[5],a[6],a[7],a[8]);
 wire[3:0] votes={3'b0,a[0]}+{3'b0,a[1]}+{3'b0,a[2]}+
 {3'b0,a[3]}+{3'b0,a[4]}+{3'b0,a[5]}+{3'b0,a[6]}+{3'b0,a[7]}+{3'b0,a[8]};
 wire cleaned=ax>=4 && ay>=4 && votes>=5;
 window3 #(W,1) dilate_window(clk,rst_n,av,ah,ad,ax,ay,cleaned,
 bv,bh,bd,bx,by,b[0],b[1],b[2],b[3],b[4],b[5],b[6],b[7],b[8]);
 wire dilated=bx>=6 && by>=6 && |b;
 window3 #(W,1) erode_window(clk,rst_n,bv,bh,bd,bx,by,dilated,
 ov,oh,od,ox,oy,c[0],c[1],c[2],c[3],c[4],c[5],c[6],c[7],c[8]);
 assign foreground=ox>=8 && oy>=8 && ox<W-1 && oy<H-1 && &c;
endmodule
module controls_measure #(parameter CLK_HZ=68493000)(
 input clk,rst_n,key_function_n,key_action_n,frame_vs,
 input edge_vs,edge_de,input[10:0] edge_x,edge_y,input edge_pixel,
 output reg[1:0] function_id,output reg[3:0] algorithm,
 output reg calibrated,output reg[7:0] span,
 output reg[15:0] width10,height10,
 output reg[10:0] bx,by,bw,bh,output reg[1:0] measure_status);
 localparam DB=CLK_HZ/50,HOLD=CLK_HZ;
 reg[1:0] fk,ak;reg fs,as_;reg[31:0] fc,ac,held;reg long_done;
 reg[1:0] requested_function;reg[3:0] requested_algorithm;
 reg fv,ev;
 reg[18:0] count;reg[10:0] xmin,xmax,ymin,ymax;reg clipped;
 reg[3:0] stable;
 wire[10:0] new_w=xmax-xmin+1'b1,new_h=ymax-ymin+1'b1;
 wire[19:0] gain=scale_gain(span);
 wire compatible=measure_status!=0 && xmin+3>=bx && bx+3>=xmin && ymin+3>=by && by+3>=ymin &&
 new_w+3>=bw && bw+3>=new_w && new_h+3>=bh && bh+3>=new_h;
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n)begin
   fk<=3;ak<=3;fs<=1;as_<=1;fc<=0;ac<=0;held<=0;long_done<=0;
   requested_function<=0;requested_algorithm<=0;function_id<=0;algorithm<=0;
   calibrated<=0;span<=160;fv<=0;ev<=0;count<=0;clipped<=0;
   xmin<=1023;xmax<=0;ymin<=599;ymax<=0;
   bx<=0;by<=0;bw<=0;bh<=0;width10<=0;height10<=0;measure_status<=0;stable<=0;
  end else begin
   fk<={fk[0],key_function_n};ak<={ak[0],key_action_n};fv<=frame_vs;ev<=edge_vs;
   if(fk[1]==fs)fc<=0;
   else if(fc==DB-1)begin
    fc<=0;fs<=fk[1];
    if(!fk[1])requested_function<=requested_function==2?0:requested_function+1'b1;
   end else fc<=fc+1'b1;
   if(ak[1]==as_)ac<=0;
   else if(ac==DB-1)begin
    ac<=0;as_<=ak[1];
    if(ak[1] && !long_done)begin
     if(function_id==0)requested_algorithm<=requested_algorithm==8?0:requested_algorithm+1'b1;
     else if(function_id==1 && !calibrated)span<=span>=240?120:span+2;
    end
   end else ac<=ac+1'b1;
   if(as_)begin held<=0;long_done<=0;end
   else if(!long_done)begin
    if(held==HOLD-1)begin
     held<=0;long_done<=1;
     if(function_id==1)begin calibrated<=~calibrated;stable<=0;measure_status<=0;end
    end else held<=held+1'b1;
   end
   if(frame_vs && !fv)begin function_id<=requested_function;algorithm<=requested_algorithm;end
   if(edge_de && edge_x>=160 && edge_x<864 && edge_y>=12 && edge_y<440 && edge_pixel)begin
    count<=count+1'b1;
    if(edge_x<xmin)xmin<=edge_x;if(edge_x>xmax)xmax<=edge_x;
    if(edge_y<ymin)ymin<=edge_y;if(edge_y>ymax)ymax<=edge_y;
    if(edge_x==160 || edge_x==863 || edge_y==12 || edge_y==439)clipped<=1;
   end
   if(edge_vs && !ev)begin
    count<=0;xmin<=1023;xmax<=0;ymin<=599;ymax<=0;clipped<=0;
    if(!calibrated || count<100 || clipped)begin measure_status<=0;stable<=0;width10<=0;height10<=0;end
    else begin
     bx<=xmin;by<=ymin;bw<=new_w;bh<=new_h;
     if(!compatible)begin stable<=0;measure_status<=1;end
     else if(stable<4)begin stable<=stable+1'b1;measure_status<=1;end
     else begin
      width10<=((new_w*gain)*10)>>16;height10<=((new_h*gain)*10)>>16;
      measure_status<=2;
     end
    end
   end
  end
 end
// Generated reciprocal function appended here.
function[19:0] scale_gain;input[7:0] span;begin case(span)
8'd120:scale_gain=20'd10923;
8'd121:scale_gain=20'd10832;
8'd122:scale_gain=20'd10744;
8'd123:scale_gain=20'd10656;
8'd124:scale_gain=20'd10570;
8'd125:scale_gain=20'd10486;
8'd126:scale_gain=20'd10403;
8'd127:scale_gain=20'd10321;
8'd128:scale_gain=20'd10240;
8'd129:scale_gain=20'd10161;
8'd130:scale_gain=20'd10082;
8'd131:scale_gain=20'd10005;
8'd132:scale_gain=20'd9930;
8'd133:scale_gain=20'd9855;
8'd134:scale_gain=20'd9781;
8'd135:scale_gain=20'd9709;
8'd136:scale_gain=20'd9638;
8'd137:scale_gain=20'd9567;
8'd138:scale_gain=20'd9498;
8'd139:scale_gain=20'd9430;
8'd140:scale_gain=20'd9362;
8'd141:scale_gain=20'd9296;
8'd142:scale_gain=20'd9230;
8'd143:scale_gain=20'd9166;
8'd144:scale_gain=20'd9102;
8'd145:scale_gain=20'd9039;
8'd146:scale_gain=20'd8978;
8'd147:scale_gain=20'd8916;
8'd148:scale_gain=20'd8856;
8'd149:scale_gain=20'd8797;
8'd150:scale_gain=20'd8738;
8'd151:scale_gain=20'd8680;
8'd152:scale_gain=20'd8623;
8'd153:scale_gain=20'd8567;
8'd154:scale_gain=20'd8511;
8'd155:scale_gain=20'd8456;
8'd156:scale_gain=20'd8402;
8'd157:scale_gain=20'd8349;
8'd158:scale_gain=20'd8296;
8'd159:scale_gain=20'd8244;
8'd160:scale_gain=20'd8192;
8'd161:scale_gain=20'd8141;
8'd162:scale_gain=20'd8091;
8'd163:scale_gain=20'd8041;
8'd164:scale_gain=20'd7992;
8'd165:scale_gain=20'd7944;
8'd166:scale_gain=20'd7896;
8'd167:scale_gain=20'd7849;
8'd168:scale_gain=20'd7802;
8'd169:scale_gain=20'd7756;
8'd170:scale_gain=20'd7710;
8'd171:scale_gain=20'd7665;
8'd172:scale_gain=20'd7620;
8'd173:scale_gain=20'd7576;
8'd174:scale_gain=20'd7533;
8'd175:scale_gain=20'd7490;
8'd176:scale_gain=20'd7447;
8'd177:scale_gain=20'd7405;
8'd178:scale_gain=20'd7364;
8'd179:scale_gain=20'd7322;
8'd180:scale_gain=20'd7282;
8'd181:scale_gain=20'd7242;
8'd182:scale_gain=20'd7202;
8'd183:scale_gain=20'd7162;
8'd184:scale_gain=20'd7123;
8'd185:scale_gain=20'd7085;
8'd186:scale_gain=20'd7047;
8'd187:scale_gain=20'd7009;
8'd188:scale_gain=20'd6972;
8'd189:scale_gain=20'd6935;
8'd190:scale_gain=20'd6899;
8'd191:scale_gain=20'd6862;
8'd192:scale_gain=20'd6827;
8'd193:scale_gain=20'd6791;
8'd194:scale_gain=20'd6756;
8'd195:scale_gain=20'd6722;
8'd196:scale_gain=20'd6687;
8'd197:scale_gain=20'd6653;
8'd198:scale_gain=20'd6620;
8'd199:scale_gain=20'd6587;
8'd200:scale_gain=20'd6554;
8'd201:scale_gain=20'd6521;
8'd202:scale_gain=20'd6489;
8'd203:scale_gain=20'd6457;
8'd204:scale_gain=20'd6425;
8'd205:scale_gain=20'd6394;
8'd206:scale_gain=20'd6363;
8'd207:scale_gain=20'd6332;
8'd208:scale_gain=20'd6302;
8'd209:scale_gain=20'd6271;
8'd210:scale_gain=20'd6242;
8'd211:scale_gain=20'd6212;
8'd212:scale_gain=20'd6183;
8'd213:scale_gain=20'd6154;
8'd214:scale_gain=20'd6125;
8'd215:scale_gain=20'd6096;
8'd216:scale_gain=20'd6068;
8'd217:scale_gain=20'd6040;
8'd218:scale_gain=20'd6012;
8'd219:scale_gain=20'd5985;
8'd220:scale_gain=20'd5958;
8'd221:scale_gain=20'd5931;
8'd222:scale_gain=20'd5904;
8'd223:scale_gain=20'd5878;
8'd224:scale_gain=20'd5851;
8'd225:scale_gain=20'd5825;
8'd226:scale_gain=20'd5800;
8'd227:scale_gain=20'd5774;
8'd228:scale_gain=20'd5749;
8'd229:scale_gain=20'd5724;
8'd230:scale_gain=20'd5699;
8'd231:scale_gain=20'd5674;
8'd232:scale_gain=20'd5650;
8'd233:scale_gain=20'd5625;
8'd234:scale_gain=20'd5601;
8'd235:scale_gain=20'd5578;
8'd236:scale_gain=20'd5554;
8'd237:scale_gain=20'd5530;
8'd238:scale_gain=20'd5507;
8'd239:scale_gain=20'd5484;
8'd240:scale_gain=20'd5461;
default:scale_gain=20'd8192;endcase end endfunction
endmodule
// Exact fixed-point connectivity on a captured frame. Shared by Canny hysteresis
// (8-connected weak pixels seeded by strong pixels) and hole fill (4-connected
// background seeded at the border). No iteration cap or incomplete result.
module frame_connect #(parameter W=1024,H=600,FRONT_LUT_WORDS=0)(
 input clk,rst_n,enable,refresh,hole_mode,
 input cap_vs,cap_de,input[10:0] cap_x,cap_y,input candidate_in,seed_in,
 input disp_vs,disp_hs,disp_de,input[10:0] disp_x,disp_y,
 output reg ov,oh,od,output reg[10:0] ox,oy,output pixel,
 output ready,busy);
 localparam PIXELS=W*H, WORDS=(PIXELS+19)/20;
 localparam IDLE=0,WAIT_FRAME=1,CAPTURE=2,REQ=3,CHECK=4,NREQ=5,NCHECK=6,WRITE=7,ADVANCE=8,DONE=9,COPY=10,COPY_LAST=11;
 reg[3:0] state;
 reg[19:0] candidates[0:WORDS-1],reached[0:WORDS-1];
 reg[19:0] cap_candidate,cap_seed;
 reg[19:0] cq,rq,cur_word,display_word;
 // Full panel uses block RAM; the optional LUT tail is exercised by simulation.
 localparam FRONT_BLOCK_WORDS=WORDS>FRONT_LUT_WORDS ? WORDS-FRONT_LUT_WORDS : WORDS;
 (* ram_style="bram" *) reg[19:0] front_block[0:FRONT_BLOCK_WORDS-1];
 reg[14:0] copy_ptr,copy_delayed;
 reg copy_valid,has_front,disp_vs_d;
 wire publish_start=disp_vs && !disp_vs_d;
 reg[4:0] display_bit;
 reg[19:0] idx;
 reg[10:0] ix,iy;
 reg[3:0] neighbor;
 reg reverse,changed,cap_vs_d,mode_latched;
 wire frame_start=cap_vs && !cap_vs_d;
 reg[14:0] cap_word_addr,current_word_addr;
 reg[4:0] cap_bit,current_bit;
 wire cap_last=cap_y==H-1 && cap_x==W-1;
 wire cap_write=state==CAPTURE && cap_de && cap_x<W && cap_y<H;
 wire cap_flush=cap_write && (cap_bit==19 || cap_last);
 reg[19:0] next_candidates,next_seeds;
 always @* begin
  next_candidates=cap_bit==0 ? 0 : cap_candidate;
  next_seeds=cap_bit==0 ? 0 : cap_seed;
  next_candidates[cap_bit]=candidate_in;
  next_seeds[cap_bit]=seed_in && candidate_in;
 end
 reg neighbor_valid;
 reg[19:0] neighbor_idx;
 always @* begin
  neighbor_valid=1;neighbor_idx=idx;
  case(neighbor)
   0:begin neighbor_valid=ix>0;neighbor_idx=idx-1;end
   1:begin neighbor_valid=ix<W-1;neighbor_idx=idx+1;end
   2:begin neighbor_valid=iy>0;neighbor_idx=idx-W;end
   3:begin neighbor_valid=iy<H-1;neighbor_idx=idx+W;end
   4:begin neighbor_valid=ix>0 && iy>0;neighbor_idx=idx-W-1;end
   5:begin neighbor_valid=ix<W-1 && iy>0;neighbor_idx=idx-W+1;end
   6:begin neighbor_valid=ix>0 && iy<H-1;neighbor_idx=idx+W-1;end
   7:begin neighbor_valid=ix<W-1 && iy<H-1;neighbor_idx=idx+W+1;end
   default:neighbor_valid=0;
  endcase
 end
 // Track packed addresses incrementally: division/modulo by 20 in the
 // RAM-address path otherwise forms an overlong combinational divider.
 reg[14:0] neighbor_word_addr;
 reg signed[6:0] neighbor_bit_sum;
 reg[4:0] neighbor_bit;
 always @* begin
  neighbor_word_addr=current_word_addr;
  neighbor_bit_sum=$signed({2'b0,current_bit});
  case(neighbor)
   0:neighbor_bit_sum=$signed({2'b0,current_bit})-1;
   1:neighbor_bit_sum=$signed({2'b0,current_bit})+1;
   2:begin neighbor_word_addr=current_word_addr-W/20;neighbor_bit_sum=$signed({2'b0,current_bit})-W%20;end
   3:begin neighbor_word_addr=current_word_addr+W/20;neighbor_bit_sum=$signed({2'b0,current_bit})+W%20;end
   4:begin neighbor_word_addr=current_word_addr-(W+1)/20;neighbor_bit_sum=$signed({2'b0,current_bit})-(W+1)%20;end
   5:begin neighbor_word_addr=current_word_addr-(W-1)/20;neighbor_bit_sum=$signed({2'b0,current_bit})-(W-1)%20;end
   6:begin neighbor_word_addr=current_word_addr+(W-1)/20;neighbor_bit_sum=$signed({2'b0,current_bit})+(W-1)%20;end
   7:begin neighbor_word_addr=current_word_addr+(W+1)/20;neighbor_bit_sum=$signed({2'b0,current_bit})+(W+1)%20;end
  endcase
  if(neighbor_bit_sum<0)begin neighbor_word_addr=neighbor_word_addr-1'b1;neighbor_bit=neighbor_bit_sum+20;end
  else if(neighbor_bit_sum>=20)begin neighbor_word_addr=neighbor_word_addr+1'b1;neighbor_bit=neighbor_bit_sum-20;end
  else neighbor_bit=neighbor_bit_sum[4:0];
 end
 wire[14:0] a_addr=state==COPY ? copy_ptr : state==CAPTURE ? cap_word_addr : state==NREQ && neighbor_valid ? neighbor_word_addr : current_word_addr;
 wire[19:0] modified_word=cur_word | (20'b1<<current_bit);
 reg[14:0] display_word_counter;
 reg[4:0] display_bit_counter;
 wire display_first=disp_x==0 && disp_y==0;
 wire[14:0] display_addr=display_first ? 0 : display_word_counter;
 always @(posedge clk)begin
  if(!rst_n || publish_start)begin display_word_counter<=0;display_bit_counter<=0;end
  else if(disp_de && disp_x<W && disp_y<H)begin
   if(display_first)begin display_word_counter<=0;display_bit_counter<=1;end
   else if(display_bit_counter==19)begin display_word_counter<=display_word_counter+1'b1;display_bit_counter<=0;end
   else display_bit_counter<=display_bit_counter+1'b1;
  end
 end
 always @(posedge clk) begin
  if(cap_flush)candidates[cap_word_addr]<=next_candidates;
  if(state==REQ)cq<=candidates[current_word_addr];
  // Shared address on port A permits block RAM inference for read/modify/write.
  if(cap_flush)reached[a_addr]<=next_seeds;
  else if(state==WRITE)reached[a_addr]<=modified_word;
  rq<=reached[a_addr];
  if(disp_de && disp_x<W && disp_y<H) display_bit<=display_first ? 0 : display_bit_counter;
 end
 generate if(WORDS>FRONT_BLOCK_WORDS) begin: hybrid_front
  (* ram_style="dram" *) reg[19:0] front_lut[0:WORDS-FRONT_BLOCK_WORDS-1];
  always @(posedge clk) begin
   if(copy_valid) begin
    if(copy_delayed<FRONT_BLOCK_WORDS) front_block[copy_delayed]<=rq;
    else front_lut[copy_delayed-FRONT_BLOCK_WORDS]<=rq;
   end
   if(disp_de && disp_x<W && disp_y<H)
    if(display_addr<FRONT_BLOCK_WORDS) display_word<=front_block[display_addr];
    else display_word<=front_lut[display_addr-FRONT_BLOCK_WORDS];
  end
 end else begin: small_front
  always @(posedge clk) begin
   if(copy_valid)front_block[copy_delayed]<=rq;
   if(disp_de && disp_x<W && disp_y<H) display_word<=front_block[display_addr];
  end
 end endgenerate
 assign ready=has_front;
 assign busy=enable && !has_front;
 assign pixel=ready ? (mode_latched ? !display_word[display_bit] : display_word[display_bit]) : 0;
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin
   state<=IDLE;idx<=0;ix<=0;iy<=0;neighbor<=0;reverse<=0;changed<=0;
   cap_candidate<=0;cap_seed<=0;cur_word<=0;cap_vs_d<=0;mode_latched<=0;
   copy_ptr<=0;copy_delayed<=0;copy_valid<=0;has_front<=0;disp_vs_d<=0;
   ov<=0;oh<=0;od<=0;ox<=0;oy<=0;
   cap_word_addr<=0;cap_bit<=0;current_word_addr<=0;current_bit<=0;
  end else begin
   cap_vs_d<=cap_vs;disp_vs_d<=disp_vs;
   ov<=disp_vs;oh<=disp_hs;od<=disp_de;ox<=disp_x;oy<=disp_y;
   if(!enable)begin state<=IDLE;has_front<=0;copy_valid<=0;end
   else if(refresh || state==IDLE || hole_mode!=mode_latched)begin
    state<=WAIT_FRAME;mode_latched<=hole_mode;has_front<=0;copy_valid<=0;
   end else case(state)
    WAIT_FRAME:if(frame_start)begin state<=CAPTURE;cap_candidate<=0;cap_seed<=0;cap_word_addr<=0;cap_bit<=0;end
    CAPTURE:if(cap_write)begin
     cap_candidate<=next_candidates;cap_seed<=next_seeds;
     if(cap_bit==19)begin cap_word_addr<=cap_word_addr+1'b1;cap_bit<=0;end else cap_bit<=cap_bit+1'b1;
     if(cap_last)begin state<=REQ;idx<=0;ix<=0;iy<=0;reverse<=0;changed<=0;current_word_addr<=0;current_bit<=0;end
    end
    REQ:state<=CHECK;
    CHECK:begin
     cur_word<=rq;
     if(!cq[current_bit] || rq[current_bit])state<=ADVANCE;
     else begin neighbor<=0;state<=NREQ;end
    end
    NREQ:state<=NCHECK;
    NCHECK:begin
     if(neighbor_valid && rq[neighbor_bit])state<=WRITE;
     else if(neighbor==(mode_latched ? 3 : 7))state<=ADVANCE;
     else begin neighbor<=neighbor+1'b1;state<=NREQ;end
    end
    WRITE:begin changed<=1;state<=ADVANCE;end
    ADVANCE:begin
     if((!reverse && idx==PIXELS-1)||(reverse && idx==0))begin
      if(!changed)state<=DONE;
      else begin
       changed<=0;reverse<=~reverse;state<=REQ;
       if(!reverse)begin idx<=PIXELS-1;ix<=W-1;iy<=H-1;current_word_addr<=WORDS-1;current_bit<=(PIXELS-1)%20;end
       else begin idx<=0;ix<=0;iy<=0;current_word_addr<=0;current_bit<=0;end
      end
     end else begin
      state<=REQ;
      if(reverse)begin
       idx<=idx-1'b1;if(ix==0)begin ix<=W-1;iy<=iy-1'b1;end else ix<=ix-1'b1;
       if(current_bit==0)begin current_bit<=19;current_word_addr<=current_word_addr-1'b1;end else current_bit<=current_bit-1'b1;
      end else begin
       idx<=idx+1'b1;if(ix==W-1)begin ix<=0;iy<=iy+1'b1;end else ix<=ix+1'b1;
       if(current_bit==19)begin current_bit<=0;current_word_addr<=current_word_addr+1'b1;end else current_bit<=current_bit+1'b1;
      end
     end
    end
    // Publish 20 pixels per clock during the vertical blank. The 1024x600
    // timing provides 30912 clocks before active video; copy takes 30722.
    DONE:if(publish_start)begin state<=COPY;copy_ptr<=0;copy_valid<=0;end
    COPY:begin
     copy_delayed<=copy_ptr;copy_valid<=1;
     if(copy_ptr==WORDS-1)state<=COPY_LAST;else copy_ptr<=copy_ptr+1'b1;
    end
    COPY_LAST:begin copy_valid<=0;has_front<=1;state<=WAIT_FRAME;end
    default:state<=IDLE;
   endcase
  end
 end
endmodule
