`timescale 1ns/1ps
/////////////////////////////////////////////////////////////////////////////
// gamma_lut_v1.v —— 增益无关的固定 gamma 表
//
//   本文件由 gen_gamma_lut.py 生成, 不要手工编辑。
//   重新生成:  python gen_gamma_lut.py
//
//   表项 = round( 255 * (min(k/2,255)/255) ** (1/2.2000) ),  k = 0..511
//   语义与 raw_pattern_test.m 的 `255 * (rgb/255).^(1/2.2)` 一致, 逐项四舍五入。
//
//   索引 k 是白平衡输出的 Q8.1 表示(每像素 9 位, 低 1 位是小数):
//   带着这 1 位小数查表, 而不是先取整成 8 位, 与 MATLAB double 的实测最大差
//   从 3 降到 1。表头 k=511 对应 255.5, 钳位后 gamma 恒为 255。
//
//   只写一张表: gamma 是逐通道相同的固定函数(增益已在上游 wb_gain_3ch 乘掉),
//   写三张的唯一后果是将来改指数时漏改一份。
//
//   为什么用 case 而不是 $readmemh 读 .hex: 仿真按 cwd 解析表文件、综合按工程
//   目录解析, 两者行为不一致, 是最容易埋下"仿真过、综合不过"的坑。case 是
//   纯 Verilog-2001, 无文件依赖。default 分支是必须的, 否则综合会判 case
//   不完整而推出锁存器。
/////////////////////////////////////////////////////////////////////////////

module gamma_lut_v1
#(
	parameter	P_DEPTH	= 8,		// 每个像素的输出位宽
	parameter	IN_FRAC	= 1		// 输入索引的小数位数 (Q8.1)
)
(
	input				i_pclk,
	input				i_arstn,

	input				i_vs,
	input				i_hs,
	input				i_de,
	input				i_valid,
	input	[2*(P_DEPTH+IN_FRAC)-1:0]	i_r,	// R 平面, {高字节像素, 低字节像素}, 每像素 9 位 Q8.1
	input	[2*(P_DEPTH+IN_FRAC)-1:0]	i_g,
	input	[2*(P_DEPTH+IN_FRAC)-1:0]	i_b,

	output	reg			o_vs,
	output	reg			o_hs,
	output	reg			o_de,
	output	reg			o_valid,
	output	reg	[2*P_DEPTH-1:0]	o_r,	// 低字节像素 = 偶列, 与 raw_to_rgb 的输出约定一致
	output	reg	[2*P_DEPTH-1:0]	o_g,
	output	reg	[2*P_DEPTH-1:0]	o_b
);

localparam	IW = P_DEPTH + IN_FRAC;	// 索引位宽 = 9

	// 纯组合查表。一个 function 被调用 6 次(3 个平面 x 高/低两个像素),
	// 综合会展开成 6 份分布式 ROM; 用 function 写, 代码量是 6 段 case 的 1/6,
	// 而且将来改指数时不可能漏改其中一份。
	function [P_DEPTH-1:0] gamma8;
		input [IW-1:0] k;
		begin
			case (k)
				9'd0: gamma8 = 8'd0;
				9'd1: gamma8 = 8'd15;
				9'd2: gamma8 = 8'd21;
				9'd3: gamma8 = 8'd25;
				9'd4: gamma8 = 8'd28;
				9'd5: gamma8 = 8'd31;
				9'd6: gamma8 = 8'd34;
				9'd7: gamma8 = 8'd36;
				9'd8: gamma8 = 8'd39;
				9'd9: gamma8 = 8'd41;
				9'd10: gamma8 = 8'd43;
				9'd11: gamma8 = 8'd45;
				9'd12: gamma8 = 8'd46;
				9'd13: gamma8 = 8'd48;
				9'd14: gamma8 = 8'd50;
				9'd15: gamma8 = 8'd51;
				9'd16: gamma8 = 8'd53;
				9'd17: gamma8 = 8'd54;
				9'd18: gamma8 = 8'd56;
				9'd19: gamma8 = 8'd57;
				9'd20: gamma8 = 8'd59;
				9'd21: gamma8 = 8'd60;
				9'd22: gamma8 = 8'd61;
				9'd23: gamma8 = 8'd62;
				9'd24: gamma8 = 8'd64;
				9'd25: gamma8 = 8'd65;
				9'd26: gamma8 = 8'd66;
				9'd27: gamma8 = 8'd67;
				9'd28: gamma8 = 8'd68;
				9'd29: gamma8 = 8'd69;
				9'd30: gamma8 = 8'd70;
				9'd31: gamma8 = 8'd71;
				9'd32: gamma8 = 8'd72;
				9'd33: gamma8 = 8'd73;
				9'd34: gamma8 = 8'd74;
				9'd35: gamma8 = 8'd75;
				9'd36: gamma8 = 8'd76;
				9'd37: gamma8 = 8'd77;
				9'd38: gamma8 = 8'd78;
				9'd39: gamma8 = 8'd79;
				9'd40: gamma8 = 8'd80;
				9'd41: gamma8 = 8'd81;
				9'd42: gamma8 = 8'd82;
				9'd43: gamma8 = 8'd83;
				9'd44: gamma8 = 8'd84;
				9'd45: gamma8 = 8'd85;
				9'd46: gamma8 = 8'd85;
				9'd47: gamma8 = 8'd86;
				9'd48: gamma8 = 8'd87;
				9'd49: gamma8 = 8'd88;
				9'd50: gamma8 = 8'd89;
				9'd51: gamma8 = 8'd90;
				9'd52: gamma8 = 8'd90;
				9'd53: gamma8 = 8'd91;
				9'd54: gamma8 = 8'd92;
				9'd55: gamma8 = 8'd93;
				9'd56: gamma8 = 8'd93;
				9'd57: gamma8 = 8'd94;
				9'd58: gamma8 = 8'd95;
				9'd59: gamma8 = 8'd96;
				9'd60: gamma8 = 8'd96;
				9'd61: gamma8 = 8'd97;
				9'd62: gamma8 = 8'd98;
				9'd63: gamma8 = 8'd99;
				9'd64: gamma8 = 8'd99;
				9'd65: gamma8 = 8'd100;
				9'd66: gamma8 = 8'd101;
				9'd67: gamma8 = 8'd101;
				9'd68: gamma8 = 8'd102;
				9'd69: gamma8 = 8'd103;
				9'd70: gamma8 = 8'd103;
				9'd71: gamma8 = 8'd104;
				9'd72: gamma8 = 8'd105;
				9'd73: gamma8 = 8'd105;
				9'd74: gamma8 = 8'd106;
				9'd75: gamma8 = 8'd107;
				9'd76: gamma8 = 8'd107;
				9'd77: gamma8 = 8'd108;
				9'd78: gamma8 = 8'd109;
				9'd79: gamma8 = 8'd109;
				9'd80: gamma8 = 8'd110;
				9'd81: gamma8 = 8'd110;
				9'd82: gamma8 = 8'd111;
				9'd83: gamma8 = 8'd112;
				9'd84: gamma8 = 8'd112;
				9'd85: gamma8 = 8'd113;
				9'd86: gamma8 = 8'd114;
				9'd87: gamma8 = 8'd114;
				9'd88: gamma8 = 8'd115;
				9'd89: gamma8 = 8'd115;
				9'd90: gamma8 = 8'd116;
				9'd91: gamma8 = 8'd116;
				9'd92: gamma8 = 8'd117;
				9'd93: gamma8 = 8'd118;
				9'd94: gamma8 = 8'd118;
				9'd95: gamma8 = 8'd119;
				9'd96: gamma8 = 8'd119;
				9'd97: gamma8 = 8'd120;
				9'd98: gamma8 = 8'd120;
				9'd99: gamma8 = 8'd121;
				9'd100: gamma8 = 8'd122;
				9'd101: gamma8 = 8'd122;
				9'd102: gamma8 = 8'd123;
				9'd103: gamma8 = 8'd123;
				9'd104: gamma8 = 8'd124;
				9'd105: gamma8 = 8'd124;
				9'd106: gamma8 = 8'd125;
				9'd107: gamma8 = 8'd125;
				9'd108: gamma8 = 8'd126;
				9'd109: gamma8 = 8'd126;
				9'd110: gamma8 = 8'd127;
				9'd111: gamma8 = 8'd128;
				9'd112: gamma8 = 8'd128;
				9'd113: gamma8 = 8'd129;
				9'd114: gamma8 = 8'd129;
				9'd115: gamma8 = 8'd130;
				9'd116: gamma8 = 8'd130;
				9'd117: gamma8 = 8'd131;
				9'd118: gamma8 = 8'd131;
				9'd119: gamma8 = 8'd132;
				9'd120: gamma8 = 8'd132;
				9'd121: gamma8 = 8'd133;
				9'd122: gamma8 = 8'd133;
				9'd123: gamma8 = 8'd134;
				9'd124: gamma8 = 8'd134;
				9'd125: gamma8 = 8'd135;
				9'd126: gamma8 = 8'd135;
				9'd127: gamma8 = 8'd136;
				9'd128: gamma8 = 8'd136;
				9'd129: gamma8 = 8'd137;
				9'd130: gamma8 = 8'd137;
				9'd131: gamma8 = 8'd137;
				9'd132: gamma8 = 8'd138;
				9'd133: gamma8 = 8'd138;
				9'd134: gamma8 = 8'd139;
				9'd135: gamma8 = 8'd139;
				9'd136: gamma8 = 8'd140;
				9'd137: gamma8 = 8'd140;
				9'd138: gamma8 = 8'd141;
				9'd139: gamma8 = 8'd141;
				9'd140: gamma8 = 8'd142;
				9'd141: gamma8 = 8'd142;
				9'd142: gamma8 = 8'd143;
				9'd143: gamma8 = 8'd143;
				9'd144: gamma8 = 8'd144;
				9'd145: gamma8 = 8'd144;
				9'd146: gamma8 = 8'd144;
				9'd147: gamma8 = 8'd145;
				9'd148: gamma8 = 8'd145;
				9'd149: gamma8 = 8'd146;
				9'd150: gamma8 = 8'd146;
				9'd151: gamma8 = 8'd147;
				9'd152: gamma8 = 8'd147;
				9'd153: gamma8 = 8'd148;
				9'd154: gamma8 = 8'd148;
				9'd155: gamma8 = 8'd148;
				9'd156: gamma8 = 8'd149;
				9'd157: gamma8 = 8'd149;
				9'd158: gamma8 = 8'd150;
				9'd159: gamma8 = 8'd150;
				9'd160: gamma8 = 8'd151;
				9'd161: gamma8 = 8'd151;
				9'd162: gamma8 = 8'd151;
				9'd163: gamma8 = 8'd152;
				9'd164: gamma8 = 8'd152;
				9'd165: gamma8 = 8'd153;
				9'd166: gamma8 = 8'd153;
				9'd167: gamma8 = 8'd154;
				9'd168: gamma8 = 8'd154;
				9'd169: gamma8 = 8'd154;
				9'd170: gamma8 = 8'd155;
				9'd171: gamma8 = 8'd155;
				9'd172: gamma8 = 8'd156;
				9'd173: gamma8 = 8'd156;
				9'd174: gamma8 = 8'd156;
				9'd175: gamma8 = 8'd157;
				9'd176: gamma8 = 8'd157;
				9'd177: gamma8 = 8'd158;
				9'd178: gamma8 = 8'd158;
				9'd179: gamma8 = 8'd158;
				9'd180: gamma8 = 8'd159;
				9'd181: gamma8 = 8'd159;
				9'd182: gamma8 = 8'd160;
				9'd183: gamma8 = 8'd160;
				9'd184: gamma8 = 8'd160;
				9'd185: gamma8 = 8'd161;
				9'd186: gamma8 = 8'd161;
				9'd187: gamma8 = 8'd162;
				9'd188: gamma8 = 8'd162;
				9'd189: gamma8 = 8'd162;
				9'd190: gamma8 = 8'd163;
				9'd191: gamma8 = 8'd163;
				9'd192: gamma8 = 8'd164;
				9'd193: gamma8 = 8'd164;
				9'd194: gamma8 = 8'd164;
				9'd195: gamma8 = 8'd165;
				9'd196: gamma8 = 8'd165;
				9'd197: gamma8 = 8'd165;
				9'd198: gamma8 = 8'd166;
				9'd199: gamma8 = 8'd166;
				9'd200: gamma8 = 8'd167;
				9'd201: gamma8 = 8'd167;
				9'd202: gamma8 = 8'd167;
				9'd203: gamma8 = 8'd168;
				9'd204: gamma8 = 8'd168;
				9'd205: gamma8 = 8'd169;
				9'd206: gamma8 = 8'd169;
				9'd207: gamma8 = 8'd169;
				9'd208: gamma8 = 8'd170;
				9'd209: gamma8 = 8'd170;
				9'd210: gamma8 = 8'd170;
				9'd211: gamma8 = 8'd171;
				9'd212: gamma8 = 8'd171;
				9'd213: gamma8 = 8'd171;
				9'd214: gamma8 = 8'd172;
				9'd215: gamma8 = 8'd172;
				9'd216: gamma8 = 8'd173;
				9'd217: gamma8 = 8'd173;
				9'd218: gamma8 = 8'd173;
				9'd219: gamma8 = 8'd174;
				9'd220: gamma8 = 8'd174;
				9'd221: gamma8 = 8'd174;
				9'd222: gamma8 = 8'd175;
				9'd223: gamma8 = 8'd175;
				9'd224: gamma8 = 8'd175;
				9'd225: gamma8 = 8'd176;
				9'd226: gamma8 = 8'd176;
				9'd227: gamma8 = 8'd177;
				9'd228: gamma8 = 8'd177;
				9'd229: gamma8 = 8'd177;
				9'd230: gamma8 = 8'd178;
				9'd231: gamma8 = 8'd178;
				9'd232: gamma8 = 8'd178;
				9'd233: gamma8 = 8'd179;
				9'd234: gamma8 = 8'd179;
				9'd235: gamma8 = 8'd179;
				9'd236: gamma8 = 8'd180;
				9'd237: gamma8 = 8'd180;
				9'd238: gamma8 = 8'd180;
				9'd239: gamma8 = 8'd181;
				9'd240: gamma8 = 8'd181;
				9'd241: gamma8 = 8'd181;
				9'd242: gamma8 = 8'd182;
				9'd243: gamma8 = 8'd182;
				9'd244: gamma8 = 8'd182;
				9'd245: gamma8 = 8'd183;
				9'd246: gamma8 = 8'd183;
				9'd247: gamma8 = 8'd183;
				9'd248: gamma8 = 8'd184;
				9'd249: gamma8 = 8'd184;
				9'd250: gamma8 = 8'd184;
				9'd251: gamma8 = 8'd185;
				9'd252: gamma8 = 8'd185;
				9'd253: gamma8 = 8'd185;
				9'd254: gamma8 = 8'd186;
				9'd255: gamma8 = 8'd186;
				9'd256: gamma8 = 8'd186;
				9'd257: gamma8 = 8'd187;
				9'd258: gamma8 = 8'd187;
				9'd259: gamma8 = 8'd187;
				9'd260: gamma8 = 8'd188;
				9'd261: gamma8 = 8'd188;
				9'd262: gamma8 = 8'd188;
				9'd263: gamma8 = 8'd189;
				9'd264: gamma8 = 8'd189;
				9'd265: gamma8 = 8'd189;
				9'd266: gamma8 = 8'd190;
				9'd267: gamma8 = 8'd190;
				9'd268: gamma8 = 8'd190;
				9'd269: gamma8 = 8'd191;
				9'd270: gamma8 = 8'd191;
				9'd271: gamma8 = 8'd191;
				9'd272: gamma8 = 8'd192;
				9'd273: gamma8 = 8'd192;
				9'd274: gamma8 = 8'd192;
				9'd275: gamma8 = 8'd193;
				9'd276: gamma8 = 8'd193;
				9'd277: gamma8 = 8'd193;
				9'd278: gamma8 = 8'd194;
				9'd279: gamma8 = 8'd194;
				9'd280: gamma8 = 8'd194;
				9'd281: gamma8 = 8'd194;
				9'd282: gamma8 = 8'd195;
				9'd283: gamma8 = 8'd195;
				9'd284: gamma8 = 8'd195;
				9'd285: gamma8 = 8'd196;
				9'd286: gamma8 = 8'd196;
				9'd287: gamma8 = 8'd196;
				9'd288: gamma8 = 8'd197;
				9'd289: gamma8 = 8'd197;
				9'd290: gamma8 = 8'd197;
				9'd291: gamma8 = 8'd198;
				9'd292: gamma8 = 8'd198;
				9'd293: gamma8 = 8'd198;
				9'd294: gamma8 = 8'd199;
				9'd295: gamma8 = 8'd199;
				9'd296: gamma8 = 8'd199;
				9'd297: gamma8 = 8'd199;
				9'd298: gamma8 = 8'd200;
				9'd299: gamma8 = 8'd200;
				9'd300: gamma8 = 8'd200;
				9'd301: gamma8 = 8'd201;
				9'd302: gamma8 = 8'd201;
				9'd303: gamma8 = 8'd201;
				9'd304: gamma8 = 8'd202;
				9'd305: gamma8 = 8'd202;
				9'd306: gamma8 = 8'd202;
				9'd307: gamma8 = 8'd202;
				9'd308: gamma8 = 8'd203;
				9'd309: gamma8 = 8'd203;
				9'd310: gamma8 = 8'd203;
				9'd311: gamma8 = 8'd204;
				9'd312: gamma8 = 8'd204;
				9'd313: gamma8 = 8'd204;
				9'd314: gamma8 = 8'd205;
				9'd315: gamma8 = 8'd205;
				9'd316: gamma8 = 8'd205;
				9'd317: gamma8 = 8'd205;
				9'd318: gamma8 = 8'd206;
				9'd319: gamma8 = 8'd206;
				9'd320: gamma8 = 8'd206;
				9'd321: gamma8 = 8'd207;
				9'd322: gamma8 = 8'd207;
				9'd323: gamma8 = 8'd207;
				9'd324: gamma8 = 8'd207;
				9'd325: gamma8 = 8'd208;
				9'd326: gamma8 = 8'd208;
				9'd327: gamma8 = 8'd208;
				9'd328: gamma8 = 8'd209;
				9'd329: gamma8 = 8'd209;
				9'd330: gamma8 = 8'd209;
				9'd331: gamma8 = 8'd210;
				9'd332: gamma8 = 8'd210;
				9'd333: gamma8 = 8'd210;
				9'd334: gamma8 = 8'd210;
				9'd335: gamma8 = 8'd211;
				9'd336: gamma8 = 8'd211;
				9'd337: gamma8 = 8'd211;
				9'd338: gamma8 = 8'd212;
				9'd339: gamma8 = 8'd212;
				9'd340: gamma8 = 8'd212;
				9'd341: gamma8 = 8'd212;
				9'd342: gamma8 = 8'd213;
				9'd343: gamma8 = 8'd213;
				9'd344: gamma8 = 8'd213;
				9'd345: gamma8 = 8'd213;
				9'd346: gamma8 = 8'd214;
				9'd347: gamma8 = 8'd214;
				9'd348: gamma8 = 8'd214;
				9'd349: gamma8 = 8'd215;
				9'd350: gamma8 = 8'd215;
				9'd351: gamma8 = 8'd215;
				9'd352: gamma8 = 8'd215;
				9'd353: gamma8 = 8'd216;
				9'd354: gamma8 = 8'd216;
				9'd355: gamma8 = 8'd216;
				9'd356: gamma8 = 8'd217;
				9'd357: gamma8 = 8'd217;
				9'd358: gamma8 = 8'd217;
				9'd359: gamma8 = 8'd217;
				9'd360: gamma8 = 8'd218;
				9'd361: gamma8 = 8'd218;
				9'd362: gamma8 = 8'd218;
				9'd363: gamma8 = 8'd218;
				9'd364: gamma8 = 8'd219;
				9'd365: gamma8 = 8'd219;
				9'd366: gamma8 = 8'd219;
				9'd367: gamma8 = 8'd220;
				9'd368: gamma8 = 8'd220;
				9'd369: gamma8 = 8'd220;
				9'd370: gamma8 = 8'd220;
				9'd371: gamma8 = 8'd221;
				9'd372: gamma8 = 8'd221;
				9'd373: gamma8 = 8'd221;
				9'd374: gamma8 = 8'd221;
				9'd375: gamma8 = 8'd222;
				9'd376: gamma8 = 8'd222;
				9'd377: gamma8 = 8'd222;
				9'd378: gamma8 = 8'd223;
				9'd379: gamma8 = 8'd223;
				9'd380: gamma8 = 8'd223;
				9'd381: gamma8 = 8'd223;
				9'd382: gamma8 = 8'd224;
				9'd383: gamma8 = 8'd224;
				9'd384: gamma8 = 8'd224;
				9'd385: gamma8 = 8'd224;
				9'd386: gamma8 = 8'd225;
				9'd387: gamma8 = 8'd225;
				9'd388: gamma8 = 8'd225;
				9'd389: gamma8 = 8'd225;
				9'd390: gamma8 = 8'd226;
				9'd391: gamma8 = 8'd226;
				9'd392: gamma8 = 8'd226;
				9'd393: gamma8 = 8'd227;
				9'd394: gamma8 = 8'd227;
				9'd395: gamma8 = 8'd227;
				9'd396: gamma8 = 8'd227;
				9'd397: gamma8 = 8'd228;
				9'd398: gamma8 = 8'd228;
				9'd399: gamma8 = 8'd228;
				9'd400: gamma8 = 8'd228;
				9'd401: gamma8 = 8'd229;
				9'd402: gamma8 = 8'd229;
				9'd403: gamma8 = 8'd229;
				9'd404: gamma8 = 8'd229;
				9'd405: gamma8 = 8'd230;
				9'd406: gamma8 = 8'd230;
				9'd407: gamma8 = 8'd230;
				9'd408: gamma8 = 8'd230;
				9'd409: gamma8 = 8'd231;
				9'd410: gamma8 = 8'd231;
				9'd411: gamma8 = 8'd231;
				9'd412: gamma8 = 8'd231;
				9'd413: gamma8 = 8'd232;
				9'd414: gamma8 = 8'd232;
				9'd415: gamma8 = 8'd232;
				9'd416: gamma8 = 8'd232;
				9'd417: gamma8 = 8'd233;
				9'd418: gamma8 = 8'd233;
				9'd419: gamma8 = 8'd233;
				9'd420: gamma8 = 8'd233;
				9'd421: gamma8 = 8'd234;
				9'd422: gamma8 = 8'd234;
				9'd423: gamma8 = 8'd234;
				9'd424: gamma8 = 8'd234;
				9'd425: gamma8 = 8'd235;
				9'd426: gamma8 = 8'd235;
				9'd427: gamma8 = 8'd235;
				9'd428: gamma8 = 8'd235;
				9'd429: gamma8 = 8'd236;
				9'd430: gamma8 = 8'd236;
				9'd431: gamma8 = 8'd236;
				9'd432: gamma8 = 8'd236;
				9'd433: gamma8 = 8'd237;
				9'd434: gamma8 = 8'd237;
				9'd435: gamma8 = 8'd237;
				9'd436: gamma8 = 8'd237;
				9'd437: gamma8 = 8'd238;
				9'd438: gamma8 = 8'd238;
				9'd439: gamma8 = 8'd238;
				9'd440: gamma8 = 8'd238;
				9'd441: gamma8 = 8'd239;
				9'd442: gamma8 = 8'd239;
				9'd443: gamma8 = 8'd239;
				9'd444: gamma8 = 8'd239;
				9'd445: gamma8 = 8'd240;
				9'd446: gamma8 = 8'd240;
				9'd447: gamma8 = 8'd240;
				9'd448: gamma8 = 8'd240;
				9'd449: gamma8 = 8'd241;
				9'd450: gamma8 = 8'd241;
				9'd451: gamma8 = 8'd241;
				9'd452: gamma8 = 8'd241;
				9'd453: gamma8 = 8'd242;
				9'd454: gamma8 = 8'd242;
				9'd455: gamma8 = 8'd242;
				9'd456: gamma8 = 8'd242;
				9'd457: gamma8 = 8'd243;
				9'd458: gamma8 = 8'd243;
				9'd459: gamma8 = 8'd243;
				9'd460: gamma8 = 8'd243;
				9'd461: gamma8 = 8'd244;
				9'd462: gamma8 = 8'd244;
				9'd463: gamma8 = 8'd244;
				9'd464: gamma8 = 8'd244;
				9'd465: gamma8 = 8'd245;
				9'd466: gamma8 = 8'd245;
				9'd467: gamma8 = 8'd245;
				9'd468: gamma8 = 8'd245;
				9'd469: gamma8 = 8'd245;
				9'd470: gamma8 = 8'd246;
				9'd471: gamma8 = 8'd246;
				9'd472: gamma8 = 8'd246;
				9'd473: gamma8 = 8'd246;
				9'd474: gamma8 = 8'd247;
				9'd475: gamma8 = 8'd247;
				9'd476: gamma8 = 8'd247;
				9'd477: gamma8 = 8'd247;
				9'd478: gamma8 = 8'd248;
				9'd479: gamma8 = 8'd248;
				9'd480: gamma8 = 8'd248;
				9'd481: gamma8 = 8'd248;
				9'd482: gamma8 = 8'd249;
				9'd483: gamma8 = 8'd249;
				9'd484: gamma8 = 8'd249;
				9'd485: gamma8 = 8'd249;
				9'd486: gamma8 = 8'd249;
				9'd487: gamma8 = 8'd250;
				9'd488: gamma8 = 8'd250;
				9'd489: gamma8 = 8'd250;
				9'd490: gamma8 = 8'd250;
				9'd491: gamma8 = 8'd251;
				9'd492: gamma8 = 8'd251;
				9'd493: gamma8 = 8'd251;
				9'd494: gamma8 = 8'd251;
				9'd495: gamma8 = 8'd252;
				9'd496: gamma8 = 8'd252;
				9'd497: gamma8 = 8'd252;
				9'd498: gamma8 = 8'd252;
				9'd499: gamma8 = 8'd252;
				9'd500: gamma8 = 8'd253;
				9'd501: gamma8 = 8'd253;
				9'd502: gamma8 = 8'd253;
				9'd503: gamma8 = 8'd253;
				9'd504: gamma8 = 8'd254;
				9'd505: gamma8 = 8'd254;
				9'd506: gamma8 = 8'd254;
				9'd507: gamma8 = 8'd254;
				9'd508: gamma8 = 8'd255;
				9'd509: gamma8 = 8'd255;
				9'd510: gamma8 = 8'd255;
				9'd511: gamma8 = 8'd255;
				default: gamma8 = 8'd0;	// 9 位覆盖了全部 512 项, 这里是防锁存
			endcase
		end
	endfunction

	wire [P_DEPTH-1:0] w_r_lo = gamma8(i_r[  IW-1:0]);
	wire [P_DEPTH-1:0] w_r_hi = gamma8(i_r[2*IW-1:IW]);
	wire [P_DEPTH-1:0] w_g_lo = gamma8(i_g[  IW-1:0]);
	wire [P_DEPTH-1:0] w_g_hi = gamma8(i_g[2*IW-1:IW]);
	wire [P_DEPTH-1:0] w_b_lo = gamma8(i_b[  IW-1:0]);
	wire [P_DEPTH-1:0] w_b_hi = gamma8(i_b[2*IW-1:IW]);

	// 标志与数据在同一个 always 块里一起打拍; 带复位, 否则出复位是 X,
	// X 会顺着乘法传进 case 的输入。
	always @(posedge i_pclk)
	begin
		if (!i_arstn)
		begin
			o_vs	<= 1'b0;
			o_hs	<= 1'b0;
			o_de	<= 1'b0;
			o_valid	<= 1'b0;
			o_r	<= {(2*P_DEPTH){1'b0}};
			o_g	<= {(2*P_DEPTH){1'b0}};
			o_b	<= {(2*P_DEPTH){1'b0}};
		end
		else
		begin
			o_vs	<= i_vs;
			o_hs	<= i_hs;
			o_de	<= i_de;
			o_valid	<= i_valid;
			o_r	<= { w_r_hi, w_r_lo };
			o_g	<= { w_g_hi, w_g_lo };
			o_b	<= { w_b_hi, w_b_lo };
		end
	end

endmodule
