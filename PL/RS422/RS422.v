// ============================================================================
// SOURCE: RFSOC_DEMO.srcs/sources_1/RS422/RS422.v
// 功能：装配 UART 波特率使能、8N1 接收与发送，仲裁逐字节回显和上层响应。
// ============================================================================
`timescale 1ns / 1ps
// =============================================================================
// 模块: RS422
// 功能: RS422/UART 物理字节层顶层，装配波特率使能、8N1 接收器和发送器。
// 接口: rxd_valid/rxd_data 向命令解析器给出收到的字节；txd_en/txd_data 接收
//       回显、长帧 ACK 或 AD 上传字节；txd_done 表示一字节发送结束。
// 仲裁: txd_ad_flag=1 时采用上层 txd_en，否则将接收字节直接回显。
// 配置: 当前系统时钟 100 MHz，DEVIDE_CNT 对应约 115200 bit/s、16 倍过采样。
// =============================================================================
module 					RS422(
	input				clk_100Mhz  	,	// 100 MHz 系统时钟

	input 				rs422_rxd		,	//pc to fpga
	output				rs422_txd		,	//fpga to pc
	output				rs422_re		,	//default 0,depend on datasheet
	output				rs422_de		,	// 发送驱动使能，极性由收发器定义
	input               txd_en          ,
	input               txd_ad_flag   ,
	output              rxd_valid       ,
	output              txd_done        ,

	input [7:0]         txd_data        ,
	output[7:0]         rxd_data
	/* output				dapower_up		,	// 在线升级测试信号，与串口数据链无关
	output				ad_power_en1	,
	output				ad_power_en2	,
	output				ad_power_en3	 */
	);
//	assign rs422_txd = rs422_rxd;

	//assign				dapower_up	=	1;
	assign				rs422_re	=	0;// 接收使能固定为0
	assign				rs422_de	=	1;// 发送使能固定为1
/* (* keep="true" *)	assign	    ad_power_en1		=	'b1 ;
(* keep="true" *) 	assign	    ad_power_en2		=	'b1 ;
(* keep="true" *)	assign	    ad_power_en3		=	'b1 ; */

//================================================================================
// 端口与内部信号

	//wire	[7:0]	rxd_data	;	// 已接收到的并行字节
	//wire			rxd_done	;	//	鎺ュ彈done淇″彿


//================================================================================

	/* wire			clk_out1	;
	wire			locked		;

	clk_wiz_0 clk_wiz_0(
    .clk_out1		(clk_out1		),	// output 100Mhz
    .reset			('d0			),	// reset high
    .locked			(locked			),
    .clk_in1_p		(clk_125MHZ_p	),
    .clk_in1_n		(clk_125MHZ_n	));	 */


//================================================================================
// 1. 产生 UART 16 倍过采样时钟使能

wire divide_clk; // 分频相位输出，原版隐式wire，当前未使用
wire	divide_clken;
wire	clken_16bps = divide_clken;

precise_divider
#(
	//	DEVIDE_CNT = 42.94967296 * fo
	//	.DEVIDE_CNT	(32'd180082484	)	//256000bps * 16
	//	.DEVIDE_CNT	(32'd81037118	)	//128000bps * 16
	//	.DEVIDE_CNT	(32'd6648556	)	//9675bps 	* 16
		.DEVIDE_CNT	(32'd79169647	)	//115207bps * 16
//		.DEVIDE_CNT	(32'd79164837	)	//115200bps * 16
//		.DEVIDE_CNT	(32'd6597070	)	//9600bps 	* 16
)
inst_precise_divider
(
	//global
	.clk			(clk_100Mhz		),	//100MHz clock
	//user interface
	.divide_clk		(divide_clk		),
	.divide_clken	(divide_clken	)
);

//================================================================================
// 2. 将上层并行字节转换为 UART 串行数据
reg txd_valid = 1'b0;


always@(posedge clk_100Mhz)begin
    if(txd_ad_flag)begin
        txd_valid<= txd_en;
    end
    else begin
        txd_valid<=rxd_valid;
    end
end

rs422_transfer	inst_rs422_transfer
(
	//gobal clock
	.clk			(clk_100Mhz		),
	//uaer interface
	.clken_16bps	(clken_16bps	),	//clk_bps * 16
	.txd_out			(rs422_txd		),	//uart txd interface fpga2pc
	//user interface
	.txd_en			(txd_valid 		),	//uart data transfer enable
	.txd_data		(txd_data		), 	//uart transfer data
	.txd_flag		(txd_done		)	//uart data transfer done
);

//================================================================================
// 3. 将 UART 串行输入恢复为并行字节

rs422_receiver	inst_rs422_receiver
(
	//gobal clock
	.clk			(	clk_100Mhz 	),
	//uart interface
	.clken_16bps	(	clken_16bps	),	    //clk_bps * 16
	.rxd			(	rs422_rxd	),		//uart txd interface
	//user interface
	.rxd_data		(	rxd_data	),		//uart data receive
	.rxd_flag		(	rxd_valid	)	  	//uart data receive done
);

endmodule
