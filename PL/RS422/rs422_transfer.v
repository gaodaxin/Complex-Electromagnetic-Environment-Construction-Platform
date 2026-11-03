// ============================================================================
// SOURCE: RFSOC_DEMO.srcs/sources_1/RS422/rs422_transfer.v
// 功能：将并行字节转换为 LSB first UART 8N1 帧，并在停止位结束后输出完成脉冲。
// ============================================================================
`timescale 1ns/1ns
// =============================================================================
// 模块: rs422_transfer
// 功能: UART 8N1 发送器，把并行 8 bit 数据转换为 LSB first 串行数据。
// 触发: txd_en 上升沿锁定一次发送请求；clken_16bps 每 16 次推进一个 UART 位。
// 输出: 一帧包含 1 位起始、8 位数据和 1 位停止，结束时 txd_flag 给出完成脉冲。
// =============================================================================
module rs422_transfer
(
	//gobal clock
	input				clk,		//100mhz

	//uart interface
	input				clken_16bps,//clk_bps * 16
	output				txd_out,		//uart txd interface

	//user interface
	input				txd_en,		//uart data transfer enable
	input		[7:0]	txd_data,	//uart transfer data
	output	reg			txd_flag	//uart data transfer done
);
reg txd;
/**************************************************************************
..IDLE...Start...............UART DATA........................End...IDLE...
________													 ______________
        |____< D0 >< D1 >< D2 >< D3 >< D4 >< D5 >< D6 >< D7 >
		Bit0  Bit1  Bit2  Bit3  Bit4  Bit5  Bit6  Bit7  Bit8  Bit9
**************************************************************************/
//---------------------------------------
//parameter of uart transfer
localparam	T_IDLE	=	2'b0;	//test the flag to transfer data
localparam	T_SEND	=	2'b1;	//uart transfer data
reg	[1:0]	txd_state =0;			//uart transfer state
reg	[3:0]	smp_cnt =0;			//16 * clk_bps, the center for sample
reg	[3:0]	txd_cnt =0 ;			//txd data counter
localparam	SMP_TOP		=	4'd15;
localparam	SMP_CENTER	=	4'd7;

reg tx_en_buf;
reg tx_en_up;

always@(posedge clk)begin
    tx_en_buf <= txd_en;
	tx_en_up  <= txd_en &&(~tx_en_buf);
end

always@(posedge clk	)begin

	case(txd_state)
	T_IDLE:
		begin
		smp_cnt <= 0;
		txd_cnt <= 0;
		if(tx_en_up == 1)
			txd_state <= T_SEND;
		else
			txd_state <= T_IDLE;
		end
	T_SEND:
		begin
		if(clken_16bps == 1)
			begin
			smp_cnt <= smp_cnt + 1'b1;
			if(smp_cnt == SMP_TOP)
				begin
				if(txd_cnt < 4'd9)
					begin
					txd_cnt <= txd_cnt + 1'b1;
					txd_state <= T_SEND;
					end
				else
					begin
					txd_cnt <= 0;
					txd_state <= T_IDLE;
					end
				end
			else
				begin
				txd_cnt <= txd_cnt;
				txd_state <= txd_state;
				end
			end
		else
			begin
			txd_cnt <= txd_cnt;
			txd_state <= txd_state;
			end
		end
	endcase

end


//--------------------------------------
//uart 8 bit data transfer
always@(*)
begin
	if(txd_state == T_SEND)
		case(txd_cnt)
		4'd0:	txd = 0;
		4'd1:	txd = txd_data[0];
		4'd2:	txd = txd_data[1];
		4'd3:	txd = txd_data[2];
		4'd4:	txd = txd_data[3];
		4'd5:	txd = txd_data[4];
		4'd6:	txd = txd_data[5];
		4'd7:	txd = txd_data[6];
		4'd8:	txd = txd_data[7];
		4'd9:	txd = 1;
		default:txd = 1;
		endcase
	else
		txd = 1'b1;	//default state
end

assign txd_out = txd;
//-------------------------------------
//Capture the falling of data transfer over

always@(posedge clk )
begin
	if(clken_16bps == 1 &&  txd_cnt == 4'd9 && smp_cnt == SMP_TOP)	//Totally 8 data is done
		txd_flag <= 1;
	else
		txd_flag <= 0;
end

endmodule
