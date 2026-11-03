// ============================================================================
// SOURCE: RFSOC_DEMO.srcs/sources_1/RS422/da_sig_excel_decode.v
// 功能：接收并校验固定 256 字节长帧，将标量、方向图和 PRI 表展开为寄存器写事务，再执行加载/启动/停止。
// ============================================================================
`timescale 1ns / 1ps
// =============================================================================
// 模块: da_sig_excel_decode
// 功能: 接收固定 256 字节 AA55 信号产生长帧，校验后拆成现有 0x61..0x80
//       内部寄存器提交，并返回 AA 55 | seq | status 四字节 ACK。
// 类别: 0=基础参数；1=方向图（首帧 216 点、续帧 220 点）；2=PRI 表。
// 顺序: 类别 2 依赖同通道类别 0 缓存的 M；类别 1 续帧依赖首帧剩余点数。
// 动作: 最后一帧 on=FF 时先发 load，下一 100 MHz 拍再发 start，避免跨域后
//       run_ctrl 因 load 优先级吞掉同拍 start。
// 容错: 帧头、通道、类别、校验、帧尾或前置关系非法时不产生 commit，只回 ERR。
//
// 固定帧字节（零基）:
//   [0]    AA                固定帧头1
//   [1]    55                固定帧头2
//   [2]    FF                固定协议标识
//   [3]    seq               上位机流水号，原样放入 ACK[2]
//   [4]    type              当前 decoder 不使用，也不参与合法性判断
//   [5]    work              0=停止该通道，非0=写参数/允许启动
//   [6]    reserved          必须为 00，否则整帧返回 ACK ERR
//   [7]    class             0=固定参数，1=方向图，2=参差 PRI 表
//   [8]    channel           线上为 1..8，内部转换成 ch=0..7
//   [9]    on                FF 且 work!=0：提交完后 load+start；00：只暂存
//   [10]   class1_cont       类别1专用，0=首帧，非0=续帧
//   [11:31]                  当前保留/忽略
//   [32:251] class payload   类别相关载荷区
//   [252:253] checksum       ~sum(frame[2:251])，大端 16 bit
//   [254:255] CC DD          固定帧尾
//
// work/on 动作矩阵（参数命令总是在校验通过后先提交）:
//   work=0, on任意值 : 提交本帧参数后 act_stop；不会 load/start
//   work=1, on=00    : 只更新100 MHz影子/表项，不触发运行参数装载
//   work=1, on=FF    : 提交后 act_load，隔1拍 act_start
//   work=1, on其他值: 与 on=00 相同；本模块只把 FF 识别为启动
// 多类别批次应让中间帧 on=00，最后一帧 on=FF，避免表尚未收齐就启动。
//
// 类别0载荷字段（所有多字节数均为大端）:
//   字节范围   名称/单位                         内部命令及说明
//   32..33    power                           当前 FPGA 忽略
//   34..37    PW，20-bit 时间 tick             0x63，取低20位
//   38        freq_type                       0固定/1脉内/2脉间/3脉组捷变
//   39..44    fc，48-bit                       0x64，仅低32位[41:44]作为FTW
//   45..48    hop bandwidth FTW               0x65
//   49        N_hop                           0x66，host_regs 再钳到1..32
//   50..51    hop dwell tick                  0x67，16 bit
//   52        hop group size                  0x68
//   53        hop random                      0x62[7]
//   54        signal type                     0普通/1调频/2调相
//   55        FM type                         调频时作为0=LFM/1=NLFM
//   56..59    LFM bandwidth FTW               0x69
//   60..63    LFM duration tick               0x6A，取低20位
//   64        LFM direction                   0正扫/1负扫
//   65        NLFM shape                      0三角/1正弦/2锯齿
//   66        PM type                         0二相/1四相/2 P3/3 P4
//   67..68    Nc，码元宽度 tick                0x6B
//   69..70    Ncode，码长                     0x6C
//   71        CW enable                       1=连续输出，不用脉冲门
//   72        Excel PRI type                  0固定/1参差/2抖动/3滑变
//   73..76    base PRI/P0 tick                0x6E默认PRI，同时写0x70抖动P0
//   77        M，参差表有效项数                钳到1..32并缓存，不在类别0提交0x6F
//   78..82    历史参差子周期/数量              当前忽略
//   83        PRI group size                  0x7D；>1时参差映射为脉组参差
//   84..87    历史脉组周期                    当前忽略
//   88..91    N_jit，抖动墙钟周期 tick         0x7F
//   92        jitter rho                      0x71[6:0]，百分比0..50
//   93        jitter mode                     0正弦/1伪随机，送0x71[7]
//   94..97    slide Pmin tick                 0x72 sel=0
//   98..101   slide Pmax tick                 0x72 sel=1
//   102       slide step mode                 Excel 1固定/0随机，转内部0固定/1随机
//   103..106  slide step tick                 0x72 sel=2
//   107       scan switch/type                0关/1圆扫/2扇扫，送0x73
//   108..109  beam width                      当前忽略；实际幅度由类别1 G表给出
//   110..111  sector span，10-bit LUT index    0x74[19:10]
//   112..113  target，10-bit LUT index         0x74[29:20]
//   114..115  scan start，10-bit LUT index     0x74[9:0]
//   116..119  T_scan，F_tick周期数             0x75
//   120..251                                  当前保留/忽略
//
// 类别1载荷:
//   首帧 [32:33]=a0，[34:35]=a1，[36:251]=最多216个8-bit G；先提交一次
//   0x7B(a0/a1)，随后每4个G打包提交一次0x79。区间长度按1024点环形地址、首尾
//   都包含来计算，并钳到256点。若总点数>216，按通道保存剩余数量。
//   续帧 [10]!=0，[32:251]=最多220个G；不再读取a0/a1，也不再提交0x7B。
//   续帧必须跟在同通道尚有剩余点数的首帧之后，否则 ACK ERR。
//
// 类别2载荷:
//   [32+4*j : 35+4*j] 为第j项32-bit PRI tick，j=0..31。有效项数不在本帧中，
//   必须使用同通道最近类别0字节77缓存的M。展开顺序严格为一次0x6F=M，然后
//   M组 0x6D=j / 0x6E=PRI[j]；尾部固定帧补零不提交。每份M缓存消费一次，
//   因此无前置类别0或重复发送类别2都会返回 ACK ERR。
//
// 输出契约:
//   commit/cmd/ch/payload 形成一拍有效的内部寄存器写事务；接收期间可连续多拍
//   commit，但每拍只写一个命令。act_* 只在所有 commit 之后出现。busy 从认领
//   首字节保持到 ACK 最后一个字节 txd_done，供上层冻结旧协议和保持 TX mux。
//
// 常用参数名/单位速查:
//   seq       上位机流水号，仅用于把ACK和请求配对，不参与参数寻址
//   channel   线上1..8；ch是送给host_regs的零基0..7通道
//   tick      上位机已经量化好的时间整数，本模块不做物理单位换算；PW/PRI/LFM等
//             在host_regs中最终保留低20位，hop dwell/Nc保留低16位
//   FTW       32-bit DDS频率控制字，通常为floor(f*2^32/Fs)，不是直接的Hz数值
//   N_hop     捷变频点数，host_regs钳到1..32
//   Nc        单个相位码元持续的tick数；Ncode是一个码周期包含的码元个数
//   P0        抖动中心PRI；rho是相对P0的抖动深度百分数0..50
//   Pmin/max  PRI滑变上下限；step是每次变化的tick数
//   M         参差PRI表有效项数1..32，运行时只循环tab[0..M-1]
//   a0/a1     1024点环形方向图中非零区域的首/末地址，两个端点都计入
//   G         8-bit方向图幅度样值；每条0x79将4个连续G一起写入compact RAM
//   T_scan    完成360度扫描的F_tick计数，和上述接口时间tick不是同一个换算基准
//   commit    一条内部寄存器写脉冲；act_load/start/stop则是整帧后的运行控制动作
//   ACK       AA 55 seq status，其中status=00成功、01校验/格式/依赖/超时失败
// =============================================================================
module da_sig_excel_decode #(
    // 任意两个长帧字节之间允许的最大空闲 clk 周期数。ST_RX 连续这么多拍
    // 没看到 rx_valid 就判定断帧并返回 ERR。100 MHz 下 5,000,000 拍=50 ms。
    parameter [22:0] RX_GAP_CYCLES    = 23'd5000000,
    // 完成接收/提交后，到发出 ACK 第一个 AA 之前等待的 clk 周期数，用于
    // USB-RS422/RS485 半双工转换器释放发送方向。100 MHz 下 50,000 拍=0.5 ms。
    parameter [15:0] ACK_TURN_CYCLES  = 16'd50000
) (
    input wire [7:0] cfg_ready,
    input  wire        clk,               // 命令域时钟，当前设计为100 MHz
    input  wire        rst,               // 高有效同步复位，清状态和跨帧缓存
    input  wire        arm,               // 1=允许在ST_IDLE认领首字节AA
    input  wire        rx_valid,          // rx_data有效脉冲，每个UART字节一拍
    input  wire [7:0]  rx_data,           // UART接收器给出的并行字节
    input  wire        txd_done,          // UART发送器完成当前ACK字节的脉冲

    output wire        busy,              // 非ST_IDLE即为1，上层据此冻结旧协议

    output reg         commit,            // host_regs写使能，每条内部命令一拍
    output reg  [7:0]  cmd,               // 内部命令码0x61..0x7F
    output reg  [2:0]  ch,                // 内部零基通道号0..7
    output reg  [63:0] payload,           // 命令载荷右对齐到低位

    output reg         act_load,          // 当前ch参数装载动作，单拍
    output reg         act_start,         // 当前ch启动动作，晚于load一拍
    output reg         act_stop,          // work=0产生的当前ch停止动作

    output reg         ack_en,             // ACK当前字节发送请求/保持
    output reg  [7:0]  ack_data            // AA、55、seq、status依次输出
);

// =============================================================================
// 1. 状态、帧缓存和跨帧上下文
// =============================================================================
localparam [2:0] ST_IDLE = 3'd0;       // 等待 arm && AA
localparam [2:0] ST_RX   = 3'd1;       // 顺序收满剩余 255 字节
localparam [2:0] ST_CHK  = 3'd2;       // 集中校验并计算本类别提交次数
localparam [2:0] ST_GO   = 3'd3;       // 每拍展开一个内部命令
localparam [2:0] ST_ACK  = 3'd4;       // 依 txd_done 发送四个 ACK 字节
localparam [2:0] ST_ACK_END = 3'd5;   // 等待 ACK 状态字真正发完
localparam [2:0] ST_ACK_WAIT = 3'd6;  // 半双工收→发方向转换保护时间
localparam [2:0] ST_START = 3'd7;     // load 后隔拍产生 start
// 计数器从 0 数到 CYCLES-1，共等待 CYCLES 拍。两个参数都允许设为 1；
// 状态机中的“<=1”分支会直接越过计数，方便仿真缩短等待时间。
localparam [22:0] RX_GAP_LAST = RX_GAP_CYCLES - 23'd1;
localparam [15:0] ACK_TURN_LAST = ACK_TURN_CYCLES - 16'd1;

reg [1:0] start_wait=0,finish_wait=0;
reg [2:0]  st;                       // 当前主状态，编码见上面的 ST_*
reg [22:0] rx_gap_cnt;               // 相邻已消费字节间的空闲拍计数
// 保留处理/ACK 忙窗内到达的下一长帧前缀；接收状态每拍可取一个字节。
// 64 字节覆盖 115200 baud 下约 5.56 ms，长于正常计算和 ACK 忙窗。
// 主机仍须逐帧等待 ACK，半双工线路上的电气碰撞不能靠缓存解决。
(* ram_style = "distributed" *) reg [7:0] rx_pending [0:63];
reg [5:0] rx_pending_wr, rx_pending_rd;
reg [6:0] rx_pending_count;
reg rx_pending_overflow;
wire rx_can_take = (st == ST_RX) || ((st == ST_IDLE) && arm);
wire rx_pending_pop = (rx_pending_count != 0) && rx_can_take;
wire rx_pending_push = rx_valid && ((rx_pending_count != 0) ||
                       (!rx_can_take && (st != ST_IDLE)));
wire rx_pending_write = rx_pending_push && ((rx_pending_count < 64) || rx_pending_pop);
wire rx_byte_valid = rx_can_take && ((rx_pending_count != 0) || rx_valid);
wire [7:0] rx_byte_data = (rx_pending_count != 0) ? rx_pending[rx_pending_rd] : rx_data;

always @(posedge clk) begin
    if (rst) begin
        rx_pending_wr <= 0;
        rx_pending_rd <= 0;
        rx_pending_count <= 0;
        rx_pending_overflow <= 0;
    end else begin
        if (rx_pending_write) begin
            rx_pending[rx_pending_wr] <= rx_data;
            rx_pending_wr <= rx_pending_wr + 1'b1;
        end
        if (rx_pending_pop)
            rx_pending_rd <= rx_pending_rd + 1'b1;
        case ({rx_pending_write, rx_pending_pop})
            2'b10: rx_pending_count <= rx_pending_count + 1'b1;
            2'b01: rx_pending_count <= rx_pending_count - 1'b1;
            default: ;
        endcase
        // 溢出使受损帧返回 ERR；不能把少字节的帧标记为成功。
        if ((st == ST_CHK) ||
            ((st == ST_RX) && !rx_byte_valid && (rx_gap_cnt == RX_GAP_LAST)) ||
            ((st == ST_IDLE) && (rx_pending_count == 0)))
            rx_pending_overflow <= 0;
        if (rx_pending_push && !rx_pending_write)
            rx_pending_overflow <= 1;
    end
end
reg [7:0]  xl_mem [0:255];           // 完整原始帧；ST_GO 随机读取载荷字段
reg [8:0]  xl_idx;                   // 0..255 接收下标，9 bit 防止回绕歧义
reg [15:0] xl_sum;                   // 字节 2..251 的模 2^16 累加和
reg [6:0]  step;                     // 当前内部 commit 步
reg [6:0]  step_max;                 // 动作拍编号，不是最后 commit 编号
reg [8:0]  g_off;                    // 本类别1帧已消费的 G 字节数
reg [4:0]  pri_i;                    // 类别2当前 PRI 表下标 0..31
// 协议没有在类别2重复携带 M，因此类别0先按通道缓存 M，类别2消费后清 valid。
reg [5:0]  pri_m_cache [0:7];
reg        pri_m_valid [0:7];
// 类别1续帧没有 a0/a1，因此首帧按区间长度缓存尚未接收的 G 点数。
reg [8:0]  g_left      [0:7];
reg        g_left_valid[0:7];
reg [1:0]  ack_i;                    // ACK字节阶段：0=AA，1=55，2=seq，3=status
reg        ack_ok;                   // 本帧最终状态：1→status=00，0→status=01
reg        need_start;               // 锁存 work!=0 && on==FF，提交后执行 load/start
reg        need_stop;                // 锁存 work==0，提交后执行 stop
reg [15:0] ack_turn_cnt;              // ST_ACK_WAIT 中的总线换向保护计数
integer    cache_i;                   // 仅用于复位时遍历 8 路跨帧缓存

// step/step_max 的约定（这是理解 ST_GO 的关键）:
//   step < step_max : 本拍 commit=1，按 step 生成一条内部命令；
//   step = step_max : 不再 commit，本拍转去执行 load/start/stop 或发送 ACK。
// 因而 step_max 恰好等于“本帧 commit 总数”，而不是最后一条命令的下标：
//   类别0             25
//   类别1首帧         1 + ceil(g_points/4)，多出的 1 是 0x7B
//   类别1续帧         ceil(g_points/4)
//   类别2             1 + 2*M，分别是 0x6F 和 M 组 0x6D/0x6E

// =============================================================================
// 2. 固定偏移字段解释与类别相关步数
// =============================================================================
// ch_raw 在线上是 1..8，内部统一转为 0..7。只有 ch_valid 为真时才允许使用
// ch_w 索引缓存；相关数组访问均位于校验通过后的状态路径。
wire [7:0] cls      = xl_mem[7];
wire [7:0] ch_raw   = xl_mem[8];
wire       ch_valid = (ch_raw >= 8'd1) && (ch_raw <= 8'd8);
wire [2:0] ch_w     = ch_raw[2:0] - 3'd1;
wire       work     = (xl_mem[5] != 8'd0);  // 0=最终 stop；非0=保留/装载参数
wire       on_ff    = (xl_mem[9] == 8'hFF); // 只有 work=1 且 on=FF 才 load+start
wire [7:0] x_pri    = xl_mem[72];           // Excel侧 PRI 类型枚举
wire [7:0] x_grp    = xl_mem[83];           // PRI 脉组内脉冲数
wire [7:0] x_scan   = xl_mem[107];          // 扫描类型；0 同时表示关闭扫描

// Excel PRI 枚举和内部 host_regs.pri_type[1:0] 并非一一直接截位：
//   x_pri=0 → 0 固定 PRI
//   x_pri=1 → x_grp<=1 时为 1 脉间参差；x_grp>1 时为 2 脉组参差
//   x_pri=2 → 3 PRI 抖动
//   x_pri=3 → 内部 pri_type 仍为 0，但另置 slide_en，交给滑变模块
//   其他值 → 按固定 PRI 处理
wire [1:0] pri_map  =
    (x_pri == 8'd1) ? ((x_grp > 8'd1) ? 2'd2 : 2'd1) :
    (x_pri == 8'd2) ? 2'd3 :
                      2'd0;
wire       slide_en = (x_pri == 8'd3);
wire       slide_md = (xl_mem[102][0] == 1'b1) ? 1'b0 : 1'b1; // Excel 1 固定 → 内部 0
wire       stag     = (pri_map == 2'd2);
wire       scan_en  = (x_scan != 8'd0);
wire [1:0] sig_ty   = xl_mem[54][1:0];
wire [1:0] freq_ty  = xl_mem[38][1:0];
wire [1:0] fm_pm    = (sig_ty == 2'd1) ? xl_mem[55][1:0] :
                      (sig_ty == 2'd2) ? xl_mem[66][1:0] : 2'd0;
// 0x61 payload 位图：
//   [7]scan_en [6]cw_en [5:4]pri_type [3:2]freq_type [1:0]signal_type
wire [7:0] mode61   = {scan_en, xl_mem[71][0], pri_map, freq_ty, sig_ty};
// 0x62 payload 位图：
//   [7]hop_rand [6:5]phase_mode(长帧固定00) [4:3]NLFM形状
//   [2]LFM负扫 [1:0]FM/PM子类型
wire [7:0] mode62   = {xl_mem[53][0], 2'b00, xl_mem[65][1:0], xl_mem[64][0], fm_pm};
// 0x7C payload 位图：
//   [4]slide_en [3]slide_mode(0固定/1随机) [2]脉组参差标志 [1:0]保留00
wire [7:0] mode7c   = {3'b000, slide_en, slide_md, stag, 2'b00};

// 类别1方向图区间允许跨 1023→0 回绕。硬件 compact RAM 最多保存 256 点，
// 因此即使上位机给出更长区间也钳到 256，首帧/续帧分别装 216/220 点。
wire [9:0]  a0_w    = {xl_mem[32][1:0], xl_mem[33]}; // 非零方向图区起点0..1023
wire [9:0]  a1_w    = {xl_mem[34][1:0], xl_mem[35]}; // 非零方向图区终点0..1023
wire        wrap_g  = (a1_w < a0_w);                // 例：a0=1000,a1=23跨零点
wire [10:0] n_g_raw = wrap_g ?
    (11'd1024 - {1'b0, a0_w} + {1'b0, a1_w} + 11'd1) :
    ({1'b0, a1_w} - {1'b0, a0_w} + 11'd1);
wire [8:0]  n_g     = (n_g_raw > 11'd256) ? 9'd256 : n_g_raw[8:0];
wire        cls1_cont = (xl_mem[10] != 8'd0);        // 0首帧，1续帧
// 当前帧实际取出的 G 点数。首帧前 4B 被 a0/a1 占用，故容量216；续帧220。
// g_left 是首帧校验通过时保存的“尚未装载点数”，续帧只能消费该数量。
wire [7:0]  g_points = cls1_cont ?
    ((g_left[ch_w] > 9'd220) ? 8'd220 : g_left[ch_w][7:0]) :
    ((n_g > 9'd216) ? 8'd216 : n_g[7:0]);
// 一个 0x79 commit 携带4个点，因此命令数为 ceil(g_points/4)。
wire [6:0]  g_pack_n = {1'b0, g_points[7:2]} +
                       ((g_points[1:0] != 2'd0) ? 7'd1 : 7'd0);

// 类别0把 M 钳到 1..32 后缓存；类别2不得从自己的补零载荷推断有效项数。
wire [7:0] x_pri_m_raw = xl_mem[77];
wire [5:0] x_pri_m = (x_pri_m_raw == 8'd0) ? 6'd1 :
                     (x_pri_m_raw > 8'd32)  ? 6'd32 : x_pri_m_raw[5:0];
wire [5:0] pri_m_w = pri_m_cache[ch_w]; // 类别2展开期间使用的本通道缓存副本

// 扫描角度不以度数传入，而是0..1023的环形 LUT 地址：1024格对应一整周。
wire [9:0] span_u  = {xl_mem[110][1:0], xl_mem[111]}; // 扇区跨度
wire [9:0] tgt_u   = {xl_mem[112][1:0], xl_mem[113]}; // 目标方向
wire [9:0] start_u = {xl_mem[114][1:0], xl_mem[115]}; // 扫描起点
// host_regs 的0x74位图：[29:20]target [19:10]span [9:0]start。
wire [31:0] pack74 = {2'b00, tgt_u, span_u, start_u};

// 多字节字段在线上均为大端。中心频率原字段占 6B，内部只使用低 4B FTW。
wire [31:0] u32_34  = {xl_mem[34], xl_mem[35], xl_mem[36], xl_mem[37]};     // PW
wire [31:0] u32_45  = {xl_mem[45], xl_mem[46], xl_mem[47], xl_mem[48]};     // 捷变带宽FTW
wire [31:0] u32_56  = {xl_mem[56], xl_mem[57], xl_mem[58], xl_mem[59]};     // LFM带宽FTW
wire [31:0] u32_60  = {xl_mem[60], xl_mem[61], xl_mem[62], xl_mem[63]};     // LFM时长
wire [31:0] u32_73  = {xl_mem[73], xl_mem[74], xl_mem[75], xl_mem[76]};     // 基准PRI/P0
wire [31:0] u32_88  = {xl_mem[88], xl_mem[89], xl_mem[90], xl_mem[91]};     // 抖动周期
wire [31:0] u32_94  = {xl_mem[94], xl_mem[95], xl_mem[96], xl_mem[97]};     // 滑变Pmin
wire [31:0] u32_98  = {xl_mem[98], xl_mem[99], xl_mem[100], xl_mem[101]};   // 滑变Pmax
wire [31:0] u32_103 = {xl_mem[103], xl_mem[104], xl_mem[105], xl_mem[106]}; // 滑变步长
wire [31:0] u32_116 = {xl_mem[116], xl_mem[117], xl_mem[118], xl_mem[119]}; // 扫描周期
wire [31:0] ftw64   = {xl_mem[41], xl_mem[42], xl_mem[43], xl_mem[44]};     // 中心频率6B低4B
wire [15:0] u16_50  = {xl_mem[50], xl_mem[51]}; // 脉内每个捷变点驻留时间
wire [15:0] u16_67  = {xl_mem[67], xl_mem[68]}; // 码元宽度Nc
wire [15:0] u16_69  = {xl_mem[69], xl_mem[70]}; // 码长Ncode

// 动态地址最大为 32+31*4+3=159，始终落在 32..251 的载荷区。
wire [7:0] pri_base = 8'd32 + {1'b0, pri_i, 2'b00};
wire [31:0] pri_now = {
    xl_mem[pri_base],
    xl_mem[pri_base + 8'd1],
    xl_mem[pri_base + 8'd2],
    xl_mem[pri_base + 8'd3]
};

// g_off 每个 0x79 增加 4；最后一组不足 4 点时上位机尾部补零，但 decoder 的
// commit 数仍由 ceil(g_points/4) 决定，不会继续扫描整片预留区。
wire [7:0] g_addr0 = (cls1_cont ? 8'd32 : 8'd36) + g_off[7:0];
wire [31:0] g_pack = {
    xl_mem[g_addr0],
    xl_mem[g_addr0 + 8'd1],
    xl_mem[g_addr0 + 8'd2],
    xl_mem[g_addr0 + 8'd3]
};

assign busy = (st != ST_IDLE) || (rx_pending_count != 0);

// 三个小函数把解出的字段对齐到 host_regs 统一的 64-bit payload 低位。
// pay8/pay32 只做零扩展；不会换算单位，也不会做饱和。host_regs 再按目标
// 寄存器宽度取 payload[7:0]、[15:0]、[19:0] 或 [31:0]。
function [63:0] pay8;
    input [7:0] v;
    begin
        pay8 = {56'd0, v};
    end
endfunction

function [63:0] pay32;
    input [31:0] v;
    begin
        pay32 = {32'd0, v};
    end
endfunction

function [63:0] pay72;
    input [1:0] sel;
    input [31:0] ticks;
    begin
        // 0x72 专用：[33:32]选择0=Pmin/1=Pmax/2=step，[31:0]携带tick。
        pay72 = {24'd0, 6'd0, sel, ticks};
    end
endfunction

// =============================================================================
// 3. 接收、校验、命令展开与 ACK 状态机
// =============================================================================
// 正常路径：
//   IDLE --收到AA--> RX --收满256B--> CHK --校验成功--> GO
//        GO --仅暂存/停止--> ACK_WAIT --> ACK --> ACK_END --> IDLE
//        GO --需启动--> START --隔拍start--> ACK_WAIT --> ACK --> ACK_END
// 异常路径：
//   RX --字节间隔超时--> ACK_WAIT(status=ERR)
//   CHK --格式/依赖失败--> ACK_WAIT(status=ERR)
// ACK_WAIT 是物理收发换向保护，ACK_END 是最后一个状态字节的发送完成保护；
// 两者目的不同，不能合并，否则可能丢首字节或让末字节中途失去发送mux。
always @(posedge clk) begin
    if (rst) begin
        // 同步复位不仅清主状态，也清“类别0→类别2”和“类别1首帧→续帧”的
        // 跨帧凭据。复位后必须重新发送前置帧，孤立类别2/续帧会返回ERR。
        st         <= ST_IDLE;
        xl_idx     <= 9'd0;
        xl_sum     <= 16'd0;
        step       <= 7'd0;
        step_max   <= 7'd0;
        g_off      <= 9'd0;
        pri_i      <= 5'd0;
        ack_i      <= 2'd0;
        ack_ok     <= 1'b0;
        need_start <= 1'b0;
        need_stop  <= 1'b0;
        rx_gap_cnt <= 23'd0;
        ack_turn_cnt <= 16'd0;
        commit     <= 1'b0;
        cmd        <= 8'd0;
        ch         <= 3'd0;
        payload    <= 64'd0;
        act_load   <= 1'b0;
        act_start  <= 1'b0;
        act_stop   <= 1'b0;
        ack_en     <= 1'b0;
        ack_data   <= 8'd0;
        for (cache_i = 0; cache_i < 8; cache_i = cache_i + 1) begin
            pri_m_cache[cache_i]  <= 6'd1;
            pri_m_valid[cache_i]  <= 1'b0;
            g_left[cache_i]       <= 9'd0;
            g_left_valid[cache_i] <= 1'b0;
        end
    end else begin
        // 所有事务型输出默认只维持一拍；具体状态按需重新置位。
        commit    <= 1'b0;
        act_load  <= 1'b0;
        act_start <= 1'b0;
        act_stop  <= 1'b0;
        ack_en    <= 1'b0;

        case (st)
            ST_IDLE: begin
                xl_idx <= 9'd0;
                xl_sum <= 16'd0;
                rx_gap_cnt <= 23'd0;
                ack_turn_cnt <= 16'd0;
                if (arm && rx_byte_valid && (rx_byte_data == 8'hAA)) begin
                    // 首字节在认领当拍写入，下一个 rxd_valid 对应 xl_mem[1]。
                    xl_mem[0] <= 8'hAA;
                    // 若帧在流水号到达前中断，ERR ACK 仍返回确定的 seq=0。
                    xl_mem[3] <= 8'h00;
                    xl_idx    <= 9'd1;
                    st        <= ST_RX;
                end
            end

            ST_RX: begin
                if (rx_byte_valid) begin
                    rx_gap_cnt <= 23'd0;
                    xl_mem[xl_idx[7:0]] <= rx_byte_data;
                    // 校验和不包括帧头、校验字段和帧尾；非阻塞赋值可保证
                    // 收到 idx=251 的同一拍把最后一个载荷字节纳入 xl_sum。
                    if ((xl_idx >= 9'd2) && (xl_idx <= 9'd251))
                        xl_sum <= xl_sum + {8'd0, rx_byte_data};
                    if (xl_idx == 9'd255)
                        st <= ST_CHK;
                    else
                        xl_idx <= xl_idx + 9'd1;
                end else if ((RX_GAP_CYCLES <= 1) || (rx_gap_cnt == RX_GAP_LAST)) begin
                    // 丢字节/断帧不能永久占住 xl_busy，否则旧短帧也会被一直冻结。
                    ack_ok       <= 1'b0;
                    need_start   <= 1'b0;
                    need_stop    <= 1'b0;
                    ack_i        <= 2'd0;
                    ack_turn_cnt <= 16'd0;
                    st           <= ST_ACK_WAIT;
                end else begin
                    rx_gap_cnt <= rx_gap_cnt + 23'd1;
                end
            end

            ST_CHK: begin
                ack_ok <= 1'b0;
                need_start <= 1'b0;
                need_stop  <= 1'b0;
                // 所有静态格式和跨帧前置关系在产生第一个 commit 之前一次检查。
                // 失败路径直接进入 ACK_WAIT，保证 host_regs 完全不被部分更新。
                // ERR 条件逐项为：AA55/FF不符、CCDD不符、反码和不符、保留字节6
                // 非零、类别不在0..2、通道不在1..8、类别1续帧没有待续点数，或
                // 类别2没有同通道类别0缓存的M。字节4和11..251中的保留区不校验。
                if (rx_pending_overflow || (xl_mem[0] != 8'hAA) || (xl_mem[1] != 8'h55) ||
                    (xl_mem[2] != 8'hFF) ||
                    (xl_mem[254] != 8'hCC) || (xl_mem[255] != 8'hDD) ||
                    ({xl_mem[252], xl_mem[253]} != (~xl_sum)) ||
                    (xl_mem[6] != 8'd0) || (cls > 8'd2) || !ch_valid ||
                    ((cls == 8'd1) && cls1_cont &&
                     (!g_left_valid[ch_w] || (g_left[ch_w] == 9'd0))) ||
                    ((cls == 8'd2) && !pri_m_valid[ch_w])) begin
                    ack_ok <= 1'b0;
                    ack_turn_cnt <= 16'd0;
                    st     <= ST_ACK_WAIT;
                    ack_i  <= 2'd0;
                end else begin
                    ack_ok     <= 1'b1;
                    need_start <= work && on_ff;
                    need_stop  <= !work;
                    ch         <= ch_w;
                    step       <= 7'd0;
                    g_off      <= 9'd0;
                    pri_i      <= 5'd0;
                    if (cls == 8'd0) begin
                        // 类别 2 必须紧跟同通道类别 0；0x6F 在类别 2 开头提交。
                        // 即使当前 PRI 不是参差，也缓存合法最小M，便于以后类别2
                        // 使用；每收到新的类别0都会覆盖该通道上一次尚未消费的M。
                        pri_m_cache[ch_w] <= x_pri_m;
                        pri_m_valid[ch_w] <= 1'b1;
                        step_max <= 7'd25;                 // 0..24 共 25 次提交，25 动作
                    end else if (cls == 8'd1) begin
                        // 首帧多一次 0x7B 区间提交；续帧只继续写 0x79。
                        step_max <= cls1_cont ?
                            g_pack_n :                     // 续帧只有 0x79
                            (g_pack_n + 7'd1);             // 首帧 0=0x7B，其后 0x79
                        if (cls1_cont) begin
                            if (g_left[ch_w] > 9'd220) begin
                                g_left[ch_w]       <= g_left[ch_w] - 9'd220;
                                g_left_valid[ch_w] <= 1'b1;
                            end else begin
                                g_left[ch_w]       <= 9'd0;
                                g_left_valid[ch_w] <= 1'b0;
                            end
                        end else if (n_g > 9'd216) begin
                            g_left[ch_w]       <= n_g - 9'd216;
                            g_left_valid[ch_w] <= 1'b1;
                        end else begin
                            g_left[ch_w]       <= 9'd0;
                            g_left_valid[ch_w] <= 1'b0;
                        end
                    end else begin
                        // 0x6F + M 组 (0x6D,0x6E)，动作拍位于 step_max。
                        // 拼接 {pri_m_w,1'b0} 等价于 2*M，最大M=32时step_max=65，
                        // 因此 step/step_max 使用7 bit，不能缩成6 bit。
                        step_max <= 7'd1 + {pri_m_w, 1'b0};
                        pri_m_valid[ch_w] <= 1'b0;
                    end
                    st <= ST_GO;
                end
            end

            ST_GO: begin
                ch <= ch_w;
                // step_max 指向“提交之后的动作拍”。这使最后一个 commit 与
                // load/stop 至少相隔一个时钟边界，宽参数先稳定再跨域采样。
                if (step == step_max) begin
                    if(finish_wait!=0) finish_wait<=finish_wait-1'b1;
                    else if(cfg_ready[ch_w]) begin
                    if (need_start) begin
                        if (cfg_ready[ch_w]) begin
                        // load/start 不能同拍：跨到 300 MHz 后 run_ctrl 会优先
                        // 处理 load 并吞掉同拍 start，导致 ACK OK 但 running 始终为 0。
                        act_load <= 1'b1; start_wait<=2;
                        st       <= ST_START;
                        end
                    end else begin
                        if (need_stop)
                            act_stop <= 1'b1;
                        ack_turn_cnt <= 16'd0;
                        st    <= ST_ACK_WAIT;
                        ack_i <= 2'd0;
                    end
                    end
                end else if (cfg_ready[ch_w]) begin
                    commit <= 1'b1;
                    if (cls == 8'd0) begin
                        // 类别0严格展开为 25 个内部命令。未被当前 FPGA 使用的
                        // Excel 字段不提交；这里也不提交 0x6F，M 留给类别2开始时。
                        case (step)
                            // 三个模式字先写，使下游知道后续数字属于哪种工作模式。
                            7'd0:  begin cmd <= 8'h61; payload <= pay8(mode61); end // 总模式/use_new
                            7'd1:  begin cmd <= 8'h62; payload <= pay8(mode62); end // 调制子类型/扫频方向
                            7'd2:  begin cmd <= 8'h7C; payload <= pay8(mode7c); end // 滑变附加模式

                            // 脉宽、载频和捷变参数。FTW已由上位机按采样率换算，
                            // decoder只重组大端字节，不再进行Hz到FTW的数学运算。
                            7'd3:  begin cmd <= 8'h63; payload <= pay32(u32_34); end       // PW
                            7'd4:  begin cmd <= 8'h64; payload <= {32'd0, ftw64}; end     // 中心频率FTW
                            7'd5:  begin cmd <= 8'h65; payload <= pay32(u32_45); end       // 捷变带宽FTW
                            7'd6:  begin cmd <= 8'h66; payload <= pay8(xl_mem[49]); end    // 捷变频点数N
                            7'd7:  begin cmd <= 8'h67; payload <= {48'd0, u16_50}; end    // 每点驻留tick
                            7'd8:  begin cmd <= 8'h68; payload <= pay8(xl_mem[52]); end    // 捷变脉组大小

                            // 调频/调相的数值参数；未选中相应调制时仍写入影子，
                            // 但运行链由0x61/0x62的模式位决定是否使用它们。
                            7'd9:  begin cmd <= 8'h69; payload <= pay32(u32_56); end       // LFM带宽FTW
                            7'd10: begin cmd <= 8'h6A; payload <= pay32(u32_60); end       // LFM时长tick
                            7'd11: begin cmd <= 8'h6B; payload <= {48'd0, u16_67}; end    // 码元宽度Nc
                            7'd12: begin cmd <= 8'h6C; payload <= {48'd0, u16_69}; end    // 码长Ncode

                            // 类别0先保证默认PRI表项0有效。类别2若存在，稍后会用
                            // 0x6D/0x6E覆盖tab[0..M-1]；这里不写0x6F。
                            7'd13: begin cmd <= 8'h6D; payload <= pay8(8'd0); end          // 选择PRI索引0
                            7'd14: begin cmd <= 8'h6E; payload <= pay32(u32_73); end       // tab[0]/默认PRI
                            7'd15: begin cmd <= 8'h7D; payload <= pay8(xl_mem[83]); end    // PRI脉组大小

                            // 抖动和滑变两套参数都写影子；实际启用项由mode61/mode7c选择。
                            7'd16: begin cmd <= 8'h70; payload <= pay32(u32_73); end       // 抖动中心P0
                            7'd17: begin cmd <= 8'h7F; payload <= pay32(u32_88); end       // 抖动周期N_jit
                            7'd18: begin cmd <= 8'h71; payload <= pay8({xl_mem[93][0], xl_mem[92][6:0]}); end // mode+rho
                            7'd19: begin cmd <= 8'h72; payload <= pay72(2'd0, u32_94); end // 滑变Pmin
                            7'd20: begin cmd <= 8'h72; payload <= pay72(2'd1, u32_98); end // 滑变Pmax
                            7'd21: begin cmd <= 8'h72; payload <= pay72(2'd2, u32_103); end // 滑变step

                            // 扫描参数：类型、三个10-bit角地址、完成一周所需F_tick数。
                            7'd22: begin cmd <= 8'h73; payload <= pay8(x_scan); end         // 0关/1圆/2扇
                            7'd23: begin cmd <= 8'h74; payload <= pay32(pack74); end        // target/span/start
                            default: begin cmd <= 8'h75; payload <= pay32(u32_116); end    // step=24,T_scan
                        endcase
                    end else if (cls == 8'd1) begin
                        if (!cls1_cont && (step == 7'd0)) begin
                            // 0x7B payload：[25:16]=a1，[9:0]=a0；host_regs收到后
                            // 保存非零区间、把compact G写指针清0，并翻转range toggle。
                            cmd     <= 8'h7B;
                            payload <= pay32({6'd0, a1_w, 6'd0, a0_w});
                        end else begin
                            // host_regs 的 0x79 写口一次携带 4 个连续 G；其内部
                            // compact 偏移随每次 commit 增加 4。
                            cmd     <= 8'h79;
                            payload <= pay32(g_pack);
                            g_off   <= g_off + 9'd4;
                        end
                    end else begin
                        // 类别2的顺序固定为 0x6F=M，然后对每项执行
                        // 0x6D=index、0x6E=value；不为不足 32 项的位置补提交。
                        if (step == 7'd0) begin
                            // host_regs用M限制运行时PRI表循环范围。先写M可保证后续
                            // 表值提交完成之前，配置影子已具有一致的有效长度。
                            cmd     <= 8'h6F;
                            payload <= pay8({2'd0, pri_m_w});
                        end else if (step[0]) begin
                            // 奇数step=1,3,...选择地址；pri_i只在下一偶数step后递增。
                            cmd     <= 8'h6D;
                            payload <= pay8({3'd0, pri_i});
                        end else begin
                            // 偶数step=2,4,...把当前4B值写入刚选择的PRI表项。
                            cmd     <= 8'h6E;
                            payload <= pay32(pri_now);
                            pri_i   <= pri_i + 5'd1;
                        end
                    end
                    if(step+7'd1==step_max) finish_wait<=2;
                    step <= step + 7'd1;
                end
            end

            ST_START: begin
                if(start_wait!=0) start_wait<=start_wait-1'b1;
                else if(cfg_ready[ch_w]) begin
                // 与 act_load 间隔一个 100 MHz 拍，使两个 toggle 跨域后仍严格有序。
                act_start    <= 1'b1;
                ack_turn_cnt <= 16'd0;
                st           <= ST_ACK_WAIT;
                ack_i        <= 2'd0;
            end

                end
            ST_ACK_WAIT: begin
                // USB-RS422/RS485 转换器在 tcdrain 后仍可能需要释放发送方向。
                // 等待线路稳定后再启动 ACK，避免首字节落在方向切换窗口内。
                if ((ACK_TURN_CYCLES <= 1) || (ack_turn_cnt == ACK_TURN_LAST)) begin
                    ack_turn_cnt <= 16'd0;
                    st <= ST_ACK;
                end else begin
                    ack_turn_cnt <= ack_turn_cnt + 16'd1;
                end
            end

            ST_ACK: begin
                // ack_en 对每个新字节重新拉高一次。发送器在 txd_en 上升沿锁定
                // 请求，因此只在前一字节 txd_done 后更新 ack_data/ack_i。
                if (ack_i == 2'd0) begin
                    ack_en   <= 1'b1;
                    ack_data <= 8'hAA;
                    ack_i    <= 2'd1;
                end else if (txd_done) begin
                    ack_en <= 1'b1;
                    case (ack_i)
                        2'd1: begin ack_data <= 8'h55;     ack_i <= 2'd2; end
                        2'd2: begin ack_data <= xl_mem[3]; ack_i <= 2'd3; end
                        default: begin
                            ack_data <= ack_ok ? 8'h00 : 8'h01;
                            // rs422_transfer 只在 txd_en 上升沿启动，且发送期间直接读取
                            // txd_data。状态字发起后必须继续占用发送 mux，直到该字节
                            // 的 txd_done 到达，否则最后一字节会被旧短帧数据替换。
                            st <= ST_ACK_END;
                        end
                    endcase
                end
            end

            ST_ACK_END: begin
                // status 发起后必须保持 busy 和发送 mux 所有权，直至停止位结束。
                if (txd_done)
                    st <= ST_IDLE;
            end

            default: st <= ST_IDLE;
        endcase
    end
end

endmodule
