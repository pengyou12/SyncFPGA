`timescale 1ns / 1ps
`include "registers.v"
module sync_event_ctrler_v3(
    input clk,
    input rst,    
    input in_master_status,
    input in_slave_status,
    input [`PORT_NUMBER - 1:0] in_sync_rx_valid,//前端buffer传输帧请求信号
    input [`SYNC_INTERVAL_WIDTH - 1:0] in_sync_interval,//Master模式下发送sync帧的间隔，单位us
    input [`LINK_DELAY_WIDTH - 1:0] in_link_delay,//链路延时，单位ns
    input [`SYNC_TIMEOUT_WIDTH - 1:0] in_sync_timeout,//Slave模式下，收不到sync帧的超时时间长度
    input [`DATA_WIDTH - 1:0] in_sync_data,//data in
    input [`CTRL_WIDTH - 1:0] in_sync_ctrl,//FF  00//数据控制位输入，表示数据帧的开始和结束
    input [`PORT_NUMBER - 1:0]in_sync_wr,//数据有效输入
    input [`PORT_NUMBER - 1:0] in_sync_udp_rdy,//next buffer//下一级buffer是否ready
    output [`PORT_NUMBER * `DATA_WIDTH - 1:0] out_sync_udp_data,//output data
    output [`PORT_NUMBER * `CTRL_WIDTH - 1:0] out_sync_udp_ctrl,//output ctrl
    output [`PORT_NUMBER - 1:0] out_sync_udp_wr,//数据有效输出
    output [`PORT_NUMBER - 1:0] out_sync_rdy,//select in port选择输入的端口
    output [`TIME_WIDTH - 1:0] out_local_clk_counter,
    output [`TIME_WIDTH - 1:0] out_global_time,
    output reg out_sync_state
    );
    reg [`SYNC_TIMEOUT_WIDTH - 1:0] timeout_slave;
    reg [`DATA_WIDTH - 1:0] rev_Headerstamp;
    reg [47:0] rev_other;
    reg [79:0] rev_originTimestamp;
    reg [63:0] rev_correctionField;
    reg [63:0] local_clk_counter;//
    reg [63:0] global_time;
	reg [47:0] mac_d_rec;
	reg [47:0] mac_s_rec;
	//-------------------------------------//
	reg [`DATA_WIDTH - 1:0] Headerstamp;
	reg [47:0] Mac_d;
	reg [47:0] Mac_s;
	reg [15:0] Ethernet_type;
	//-------------------------------------//
	reg [7:4] transportSpecific;
	reg [3:0] messageType;
	reg [7:4] reserved1;
	reg [3:0] versionPTP;
	reg [15:0] messageLength;
	reg [7:0] domainNumber;
	reg [7:0] reserved2;
	reg [15:0] flagField;
	reg [63:0] correctionField;
	reg [31:0] reserved3;
	reg [79:0] sourcePortIdentity;
	
	reg [15:0] sequenceId;
	reg [7:0] controlField;
	reg [7:0] logMessageInterval;
	reg [79:0] originTimestamp;
    reg [`DATA_WIDTH - 1:0] sync_udp_data[`PORT_NUMBER - 1:0];//output data
    reg [`CTRL_WIDTH - 1:0] sync_udp_ctrl[`PORT_NUMBER - 1:0];//output ctrl
    reg [`PORT_NUMBER - 1:0] sync_udp_wr;
    reg [`PORT_NUMBER - 1:0] sync_rdy;
    reg [15:0] current_state,next_state;
    reg [15:0] current_state_rev;
    reg [15:0] next_state_rev;
    reg [6:0] cnt_1us;
    reg [15:0] cnt_master_interval;
    reg [`DATA_WIDTH - 1:0] udp_data;
    reg [`DATA_WIDTH - 1:0] in_sync_data_r1;
    reg [`CTRL_WIDTH - 1:0] udp_ctrl;
    reg [`PORT_NUMBER - 1:0] in_port;//data from which port
	reg [79:0] time0_reg;
	reg [79:0] time1_reg;
	reg [79:0] time2_reg;
    reg rev_over;
    reg udp_wr;
    reg flag1;

    //the reg to implemetn adjust frequency algorithm//
    reg [63:0] last_global_time;
    reg flag_fre_start;
    reg [63:0] cycle;
    reg [63:0] cur_cycle;
    reg [2:0] compression;
    reg minus_flag;
    reg [63:0] master_sync_time;
    reg [63:0] slave_sync_time;
    reg [63:0] diff_time;
    reg [3:0][63:0]	diff_time_l;
    wire[63:0]	diff_time_s;
    reg	[1:0]		select_com;
    reg [63:0] cur_acc_value;
    reg [15:0] compression_index;
    reg not_first_flag;
	
    //-------------------------------------//


	localparam s0         =  16'h0001;
	localparam s1         =  16'h0002;
	localparam s2         =  16'h0004;
	localparam s3         =  16'h0008;
	localparam s4         =  16'h0010;
	localparam s5         =  16'h0020;
	localparam s6         =  16'h0040;
	localparam s7         =  16'h0080;
	localparam s8         =  16'h0100;
	localparam s9         =  16'h0200;
	localparam s0_rev         =  16'h0001;
	localparam s1_rev         =  16'h0002;
	localparam s2_rev         =  16'h0004;
	localparam s3_rev         =  16'h0008;
	localparam s4_rev         =  16'h0010;
	localparam s5_rev         =  16'h0020;
	localparam s6_rev         =  16'h0040;
	localparam s7_rev         =  16'h0080;
	localparam s8_rev         =  16'h0100;
	localparam s9_rev         =  16'h0200;
	localparam s10_rev        =  16'h0400;
	localparam s11_rev        =  16'h0800;
	localparam s12_rev        =  16'h1000;
	localparam s13_rev        =  16'h2000;
	localparam s14_rev        =  16'h4000;
	
	wire [`PORT_NUMBER - 1:0] sync_rxv1;//top buffer valid 
	wire [`PORT_NUMBER - 1:0] sync_rxv2;//top buffer valid 
	wire [`PORT_NUMBER - 1:0] sync_rxv3;//top buffer valid 
	//(* keep = "TRUE"*) 
    wire [63:0] local_clk_counter_w;
	assign out_local_clk_counter = local_clk_counter;
	assign out_global_time = global_time;//
	assign local_clk_counter_w = local_clk_counter;
	assign out_sync_rdy = sync_rdy;
	assign sync_rxv1 = in_sync_rx_valid - 1;
	assign sync_rxv2 = in_sync_rx_valid ^ sync_rxv1;
//***********************receive_state***********************//
wire reset_rev;
assign reset_rev = (rst || in_master_status);
always @ (posedge clk or posedge reset_rev)begin 
	if(reset_rev)
		current_state_rev <= s0_rev;
	else
		current_state_rev <= next_state_rev; 
 end
always @ (current_state_rev or in_sync_rx_valid or in_sync_wr or in_sync_ctrl or next_state) begin 
	next_state_rev = s0_rev; 
    case(current_state_rev)
		s0_rev:begin
			if(next_state == s0 && in_sync_rx_valid != 0)//lock receive_state when send data
				next_state_rev = s1_rev;
			else 
				next_state_rev = s0_rev;
		end
		s1_rev: begin
			if(in_sync_wr != 0 && in_sync_ctrl == 8'hFF)
				next_state_rev = s2_rev;
			else next_state_rev = s1_rev;
		end
		s2_rev: next_state_rev = s3_rev;
		s3_rev: next_state_rev = s4_rev;
		s4_rev: next_state_rev = s5_rev;
		s5_rev: next_state_rev = s6_rev;
		s6_rev: next_state_rev = s7_rev;
		s7_rev: next_state_rev = s8_rev;
		s8_rev: next_state_rev = s9_rev;
		s9_rev: next_state_rev = s10_rev;
		s10_rev: next_state_rev = s11_rev;
		s11_rev: next_state_rev = s12_rev;
		s12_rev: next_state_rev = s13_rev;
		s13_rev: next_state_rev = s14_rev;
		s14_rev: next_state_rev = s0_rev;
		//can add more states here
        default: next_state_rev = s0_rev;
    endcase
end
//generate data
always @ (posedge clk or posedge rst)begin
	if(rst)begin
		rev_Headerstamp <= 0;
		rev_correctionField <= 0;
		rev_originTimestamp <= 0;
		rev_over <= 0;
		time0_reg <= 0;
		time1_reg <= 0;
		time2_reg <= 0;
		mac_d_rec <= 0;
		mac_s_rec <= 0;
		rev_other <= 0;
		compression<= 0;
		minus_flag <= 0;
	end
	else begin
		case(next_state_rev)
			s0_rev:begin
				rev_Headerstamp <= rev_Headerstamp;
				rev_correctionField <= rev_correctionField;
				rev_originTimestamp <= rev_originTimestamp;
				rev_over <= 0;
				time0_reg <= time0_reg;
				time1_reg <= time1_reg;
				time2_reg <= time2_reg;
				mac_d_rec <= mac_d_rec;
				mac_s_rec <= mac_s_rec;
			end
			s2_rev: begin
				rev_Headerstamp <= in_sync_data;
			end
			s3_rev:begin
				mac_d_rec <= in_sync_data[63:16];
				mac_s_rec[47:32] <= in_sync_data[15:0];
			end
			s4_rev:begin
				mac_s_rec[31:0] <= in_sync_data[63:32];
			end
			s5_rev:begin
				rev_correctionField[63:48] <= in_sync_data[15:0];
			end
			s6_rev:begin
				rev_correctionField[47:0] <= in_sync_data[63:16];
			end
			s7_rev:begin
				time2_reg <= rev_correctionField[63:0]  +  in_link_delay[`LINK_DELAY_WIDTH - 1 : 0];
			end
			s8_rev:begin
			end
			s9_rev:begin
				rev_originTimestamp[79:16] <= in_sync_data;
			end
			s10_rev: begin
				rev_originTimestamp[15:0] <= in_sync_data[63:48];
				rev_other[47:0] <= in_sync_data[47:0];
			end
			s11_rev:begin
				time1_reg <= rev_originTimestamp[79:0] + time2_reg;
				time0_reg <= 690;//此处可能有错误，所以先用固定值代替 local_clk_counter - rev_Headerstamp;//+ 1
			end
			///////////////////////////////////////////////////////////////
			s12_rev:begin
				master_sync_time <= time1_reg[63:0] + {time0_reg[60:0],3'b000} + 16 - last_global_time;
				slave_sync_time <= global_time - last_global_time;
			end
			s13_rev:begin
				cur_acc_value <= 0;
				if(master_sync_time > slave_sync_time)begin
					minus_flag <= 0;
					diff_time <= master_sync_time - slave_sync_time;
				end
				else begin
					diff_time <= slave_sync_time - master_sync_time;
					minus_flag <= 1;
				end
			end
			s14_rev:begin
				rev_over <= 1;
				cycle <= {5'b0,slave_sync_time[63:5]};
			//	cycle <= slave_sync_time;

    			case(diff_time[1:0])
    				2'b01:begin
    					diff_time_l[0] <= diff_time >> 2;
    					diff_time_l[1] <= diff_time >> 2;
    					diff_time_l[2] <= diff_time >> 2;
    					diff_time_l[3] <= diff_time >> 2;
    				end
    				2'b01:begin
    					diff_time_l[0] <= diff_time >> 2 + 2'b01;
    					diff_time_l[1] <= diff_time >> 2;
    					diff_time_l[2] <= diff_time >> 2;
    					diff_time_l[3] <= diff_time >> 2;
    				end
    				2'b10:begin
    					diff_time_l[0] <= diff_time >> 2 + 2'b01;
    					diff_time_l[1] <= diff_time >> 2;
    					diff_time_l[2] <= diff_time >> 2 + 2'b01;
    					diff_time_l[3] <= diff_time >> 2;
    				end
    				2'b11:begin
    					diff_time_l[0] <= diff_time >> 2 + 2'b01;
    					diff_time_l[1] <= diff_time >> 2 + 2'b01;
    					diff_time_l[2] <= diff_time >> 2 + 2'b01;
    					diff_time_l[3] <= diff_time >> 2;
    				end
					default: begin
						diff_time_l[0] <= diff_time >> 2;
    					diff_time_l[1] <= diff_time >> 2;
    					diff_time_l[2] <= diff_time >> 2;
    					diff_time_l[3] <= diff_time >> 2;
					end
   				endcase

				// if(diff_time[0]==1)
				// begin
				// 	diff_time_l<={1'b0,diff_time[63:1]};
				// 	diff_time_h<={{1'b0,diff_time[63:1]}+1'b1};
				// end
				// else begin
				// 	diff_time_l<={1'b0,diff_time[63:1]};
				// 	diff_time_h<={1'b0,diff_time[63:1]};
				// end
				
				//wile
	/* 			if(cur_acc_value < slave_sync_time)
					begin
						cur_acc_value <= cur_acc_value + diff_time;
						compression <= compression + 1;
					end
				else begin
				
				end */
			end
			////////////////////////////////////////////////////////////////
			default:begin
				rev_Headerstamp <= rev_Headerstamp;
				rev_correctionField <= rev_correctionField;
				rev_originTimestamp <= rev_originTimestamp;
				rev_over <= 0;
				time0_reg <= time0_reg;
				time1_reg <= time1_reg;
				time2_reg <= time2_reg;
				mac_d_rec <= mac_d_rec;
				mac_s_rec <= mac_s_rec;
				rev_other <= rev_other;
			end
		endcase
	end
end
//****************************send_state******************************//
always @ (posedge clk or posedge rst)begin    
	if(rst)
		current_state <= s0;
	else
		current_state <= next_state;  
 end
always @ (current_state or rst or flag1 or rev_over or in_master_status) begin   
		next_state = s0;
    case(current_state)
		s0:if((flag1 == 1 && in_master_status == 1) || rev_over)
			next_state = s1;
		s1: next_state = s2;
		s2: next_state = s3;
		s3: next_state = s4;
		s4: next_state = s5;
		s5: next_state = s6;
		s6: next_state = s7;
		s7: next_state = s8;
		s8: next_state = s9;
		s9: next_state = s0;
        default: next_state = s0;
    endcase
end
always @ (posedge clk or posedge rst)begin
	if(rst)begin
		udp_data <= 0;
		udp_ctrl <= 0;
		udp_wr <= 0;
	end
	else begin
		case(next_state)
			s0:begin
				udp_data <= 64'h0;               
				udp_ctrl <= 8'h00;
				udp_wr <= 0;
			end
			s1:begin
				udp_data <= Headerstamp;
				udp_ctrl <= 8'hFF;
				udp_wr <= 1;
			end
			s2:begin
				udp_data <= {Mac_d[47:0],Mac_s[47 : 32]};
				udp_ctrl <= 8'h00;
				udp_wr <= 1;
			end
			s3:begin
				udp_data <= {Mac_s[31:0],Ethernet_type[15:0],transportSpecific[7:4],messageType[3:0],reserved1[7:4],versionPTP[3:0]};//
				udp_ctrl <= 8'h00;
				udp_wr <= 1;
			end
			s4:begin
				udp_data <= {messageLength[15:0],domainNumber[7:0],reserved2[7:0],flagField[15:0],correctionField[63:48]};
				udp_ctrl <= 8'h00;
				udp_wr <= 1;
			end
			s5:begin
				udp_data <= {correctionField[47:0],reserved3[31:16]};
				udp_ctrl <= 8'h00;
				udp_wr <= 1;
			end
			s6:begin
				udp_data <= {reserved3[15:0],sourcePortIdentity[79:32]};
				udp_ctrl <= 8'h00;
				udp_wr <= 1;
			end
			s7:begin
				udp_data <= {sourcePortIdentity[31:0],sequenceId[15:0],controlField[7:0],logMessageInterval[7:0]};
				udp_ctrl <= 8'h00;
				udp_wr <= 1;
			end
			s8:begin
				udp_data <= {originTimestamp[79:16]};             
				udp_ctrl <= 8'h00;
				udp_wr <= 1;
			end
			s9:begin
				// udp_data <= {originTimestamp[15:0],48'h0};
				udp_data <= {originTimestamp[15:0],rev_other};
				udp_ctrl <= 8'b00000001;
				udp_wr <= 1;
			end
			default:begin
				udp_data <= udp_data;
				udp_ctrl <= udp_ctrl;
				udp_wr <= udp_wr;
			end
		endcase
	end
end
//---------------switch------ --------//
always @ (posedge clk or posedge rst)begin
	if(rst)begin
		Headerstamp <= local_clk_counter;
		Mac_d <= 48'h011B19000000;
		Mac_s <= 48'h3C970E0F6857;
		Ethernet_type <= 16'h88F7;
		transportSpecific <= 4'h0;
		messageType <= 4'h0;
		reserved1 <= 4'h0;
		versionPTP <= 4'b0010;
		messageLength <= 16'h002C;
		domainNumber <= 8'h00;
		reserved2 <= 0;
		flagField <= 16'h0;
		correctionField <= 0;//////////?
		reserved3 <= 0;
		sourcePortIdentity <=  80'hFF0523456789ABCD;
		sequenceId <=  16'h0;
		controlField <= 8'h0;
		logMessageInterval <= 8'h7F;
		originTimestamp <= global_time;
	end
	//master
	else if (in_master_status)begin
		Headerstamp <= local_clk_counter;     //!
		Mac_d <= 48'h011B19000000;
		Mac_s <= 48'h3C970E0F6857;
		Ethernet_type <= 16'h88F7;
		transportSpecific <= 4'h0;
		messageType <= 4'h0;
		reserved1 <= 0;
		versionPTP <= 4'b0010;
		messageLength <= 16'h002C;
		domainNumber <= 8'h00;
		reserved2 <= 8'h00;
		flagField <= 16'h0000;
		correctionField <= 64'h0000000000000000;//////////?
		reserved3 <=       32'h00000000;
		sourcePortIdentity <=  80'hFF0523456789ABCD;
		sequenceId <=  16'h0000;
		controlField <= 8'h00;
		logMessageInterval <= 8'h7F;
		originTimestamp <= global_time;	// 1
	end
	//slave
	else begin
		Headerstamp <= rev_Headerstamp;
		Mac_d <= mac_d_rec;
		Mac_s <= mac_s_rec;
		Ethernet_type <= 16'h88F7;
		transportSpecific <= 4'h0;
		messageType <= 4'h0;
		reserved1 <= 0;
		versionPTP <= 4'b0010;
		messageLength <= 16'h002C;
		domainNumber <= 8'h00;
		reserved2 <= 0;
		flagField <= 16'h0;
		correctionField <= rev_correctionField + in_link_delay;//!?
		reserved3 <= 0;
		sourcePortIdentity <=  80'hFF0523456789ABCD;
		sequenceId <=  16'h0;
		controlField <= 8'h0;
		logMessageInterval <= 8'h7F;
		originTimestamp <= rev_originTimestamp;
	end
end
//data out
generate
genvar i;
	for(i = 0;i < `PORT_NUMBER;i = i + 1)begin:sync_udp_group
		always @(posedge clk or posedge rst )begin
			if (rst)begin
					sync_udp_data[i] <= 0;
					sync_udp_ctrl[i] <= 0;
					sync_udp_wr[i] <= 0;
			end
			else begin
				if((in_sync_udp_rdy[i] == 1) && ((in_master_status == 1) || ((in_slave_status == 1) && (in_port[i] == 0))))begin
					sync_udp_data[i] <= udp_data;
					sync_udp_ctrl[i] <= udp_ctrl;
					sync_udp_wr[i] <= udp_wr;
				end
				else begin
					sync_udp_data[i] <= 0;
					sync_udp_ctrl[i] <= 0;
					sync_udp_wr[i] <= 0;
				end
			end
		end
	end
endgenerate
generate
genvar j;
	for(j = 0;j < `PORT_NUMBER;j = j + 1)begin
		assign out_sync_udp_data[(j+1) * `DATA_WIDTH - 1 : j * `DATA_WIDTH] = sync_udp_data[j];
		assign out_sync_udp_ctrl[(j+1) * `CTRL_WIDTH - 1 : j * `CTRL_WIDTH] = sync_udp_ctrl[j];
		assign out_sync_udp_wr[j] = sync_udp_wr[j];
	end 
endgenerate
always @(posedge clk or posedge rst)begin
	if (rst)begin
		sync_rdy <= 0;
		in_port <= 0;
	end
	else if(current_state_rev == s0_rev && next_state == s0 && in_sync_rx_valid != 0)begin
		sync_rdy <=  sync_rxv2 & in_sync_rx_valid;
		in_port <=  sync_rxv2 & in_sync_rx_valid;
	end
	else if(next_state_rev == s10_rev)begin
		sync_rdy <= 0;
		in_port <= in_port;
	end
	else begin
		sync_rdy <= sync_rdy;
		in_port <= in_port;
	end
end
//
always @(posedge clk or posedge rst)begin
        if (rst)
            local_clk_counter <= 64'h0;
        else
            local_clk_counter <= local_clk_counter + 1; 
end
// global_time <= rev_originTimestamp[79:3] 1ns + rev_correctionField[63:3]  +  in_link_delay[LINK_DELAY_WIDTH - 1:3] + local_clk_counter –  rev_Headerstamp[DATA_WIDTH - 1:0] + 1;
always @(posedge clk or posedge rst) begin///1ns
        if (rst)begin
            global_time <= 64'h0;//unit_1ns/*****/
///////////////////////////////////////////////////////////////////////////////          
			 flag_fre_start <= 0;
            last_global_time <= 64'h0;
            cycle <= 0;
            cur_cycle <= 1;
            compression_index <= 0;
			 select_com<=1'b0;
			 not_first_flag <= 0;
		end
        else if(rev_over)begin
			if(not_first_flag)begin
				global_time <= time1_reg[63:0] + {time0_reg[60:0],3'b000} + 16;///*****/
				last_global_time <= time1_reg[63:0] + {time0_reg[60:0],3'b000} + 16;//
				flag_fre_start <= 1;
				cur_cycle <= 1;
				end
			else begin
				global_time <= time1_reg[63:0] + {time0_reg[60:0],3'b000} + 16;///*****/
				last_global_time <= time1_reg[63:0] + {time0_reg[60:0],3'b000} + 16;//
				not_first_flag <= 1;
				cur_cycle <= 1;
				flag_fre_start <= 0;
			end
		end
		else if(flag_fre_start)begin
			if(cur_cycle == cycle)begin
				cur_cycle <= 1;
				select_com<= select_com + 1;
				if(minus_flag)
					//global_time <= global_time + 8 - compression;
					global_time <= global_time + 8 - diff_time_s;
				else
					//global_time <= global_time + 8 + compression;
					global_time <= global_time + 8 + diff_time_s;
			end
			else
				begin
			    	cur_cycle <=cur_cycle + 1;
					global_time <= global_time + 8;/*****/
				end
			end

		else
            global_time <= global_time + 8; // to implement my own algorithm
////////////////////////////////////////////////////////////////////////////////////
end
assign diff_time_s = diff_time_l[select_com];
//-------------1us_counter = 125 x 8ns--------------------//
always @ (posedge clk or posedge rst)begin
	if(rst)
		cnt_1us <= 0;
	else if  (cnt_1us == 124)
		cnt_1us <= 0;
	else cnt_1us <= cnt_1us + 1;
end
////master
always @ (posedge clk or posedge rst)begin
	if(rst)
		cnt_master_interval <= 0;
	else if (cnt_master_interval == in_sync_interval)
		cnt_master_interval <= 0;
	else if (cnt_1us == 124)
		cnt_master_interval <= cnt_master_interval + 1;
end
always @ (posedge clk or posedge rst)begin
	if(rst)
		flag1 <= 0;
	else if (cnt_master_interval == in_sync_interval)
		flag1 <= 1;
	else 
		flag1 <= 0;
end
////slave
always@(posedge clk or posedge rst)begin
	if(rst)
		timeout_slave <= 0;
	else if(current_state_rev == s14_rev)
		timeout_slave <= 0;		
	else if (timeout_slave == in_sync_timeout)	
		timeout_slave <= timeout_slave;
	else if((flag1 == 1)&&(in_slave_status == 1))begin
		timeout_slave <= timeout_slave + 1;
	end
end
always@(posedge clk or posedge rst)begin
	if(rst)
		out_sync_state <= 0;
	else if (in_master_status)
		out_sync_state <= 1;
	else if (in_slave_status)
		if (timeout_slave == in_sync_timeout)
			out_sync_state <= 0;
		else 
			out_sync_state <= 1;
	else
		out_sync_state <= 0;
end
endmodule
