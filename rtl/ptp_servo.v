/*

PTP servo: steers a ptp_clock to follow a PTP master

*/

// Language: Verilog 2001

`resetall
`timescale 1ns / 1ps
`default_nettype none

/*
 * Each sample says that when the local clock read s_sample_local, the master's
 * clock read s_sample_master. The servo works on the offset local - master
 * (positive when the local clock is ahead) and drives ptp_clock's set time and
 * set period inputs. It follows the approach of linuxptp's PI servo:
 *
 * 1. First sample: stored.
 * 2. Second sample: the rate error is measured from the two samples,
 *        (local time elapsed - master time elapsed) / local time elapsed
 *    and added to the integrator. If the offset is over
 *    FIRST_STEP_THRESHOLD_NS the clock is then stepped to the master's time,
 *    and the servo goes back to 1 to measure the rate error that is left.
 * 3. Every later sample: PI control of the rate,
 *        integ = integ + KI * offset
 *        adj   = KP * offset + integ
 *    in ppb, where a positive adj slows the clock down. An offset over
 *    STEP_THRESHOLD_NS, or too large to compute (seconds parts more than 7
 *    apart), steps the clock instead and goes back to 1.
 *
 * After a step, STEP_SKIP_SAMPLES samples are ignored while the MAC's copies
 * of the time (ptp_clock_cdc) catch up with the jump. The integrator is kept,
 * so the rate correction carries over a step or a restart (holdover).
 *
 * Small offsets are only ever corrected through the rate. ptp_clock's offset
 * adjustment inputs (input_adj_*) are not used: they hold output_ts_step high
 * while active, which makes ptp_clock_cdc treat every cycle as a step.
 *
 * The rate correction is applied as a new period for ptp_clock:
 *     period = nominal period * (1 - adj * 10^-9)
 * The ptp_clock parameters must match the ones here. With FNS_WIDTH = 16, one
 * step of the period is 2^-16 ns per 6.4 ns, about 2.4 ppm, which is too coarse
 * for a servo; FNS_WIDTH = 32 gives about 0.04 ppb per step.
 *
 * Units: offsets are signed ns * 2^16, rates are signed ppb * 2^16, and KP and
 * KI are Q16 (65536 = 1.0) in ppb of rate per ns of offset. The default gains
 * are linuxptp's defaults for hardware timestamps at a 1 s Sync interval.
 * Faster Sync rates generally want a larger KP and a smaller KI.
 *
 * Measuring the rate takes a 72 cycle sequential divide. s_sample_ready is
 * low while a sample is being processed.
 */
module ptp_servo #
(
    // ptp_clock settings; these must match the ptp_clock instance
    parameter PERIOD_NS_WIDTH = 4,
    parameter FNS_WIDTH = 32,
    parameter PERIOD_NS = 4'h6,
    parameter PERIOD_FNS = 32'h66666666,
    parameter PTP_CLOCK_PIPELINE = 0,
    // PI gains, Q16, ppb per ns of offset
    parameter KP = 45875,  // 0.7
    parameter KI = 19661,  // 0.3
    // limit on the rate correction, ppb
    parameter MAX_PPB = 500000,
    // step the clock instead of slewing when the offset is over these, ns
    // (STEP_THRESHOLD_NS = 0: only step when the offset is too large to compute)
    parameter FIRST_STEP_THRESHOLD_NS = 20000,
    parameter STEP_THRESHOLD_NS = 1000000,
    // status_locked is set after LOCK_COUNT samples in a row within
    // LOCK_THRESHOLD_NS, and cleared by any sample outside it
    parameter LOCK_THRESHOLD_NS = 100,
    parameter LOCK_COUNT = 8,
    // samples ignored after a step
    parameter STEP_SKIP_SAMPLES = 1
)
(
    input  wire                        clk,
    input  wire                        rst,

    // drop the stored samples and start again from the first sample;
    // the rate correction is kept
    input  wire                        restart,

    /*
     * Samples
     */
    input  wire [95:0]                 s_sample_local,
    input  wire [95:0]                 s_sample_master,
    input  wire                        s_sample_valid,
    output wire                        s_sample_ready,

    /*
     * PTP clock
     */
    input  wire [95:0]                 ptp_ts_96,
    output wire [95:0]                 ptp_set_ts_96,
    output wire                        ptp_set_ts_96_valid,
    output wire [PERIOD_NS_WIDTH-1:0]  ptp_set_period_ns,
    output wire [FNS_WIDTH-1:0]        ptp_set_period_fns,
    output wire                        ptp_set_period_valid,

    /*
     * Status
     */
    output wire                        step,                 // pulses when the clock is stepped
    output wire [1:0]                  status_state,         // 0: first sample, 1: measuring rate, 2: tracking
    output wire                        status_locked,
    output wire [63:0]                 status_offset,        // local - master, ns * 2^16, signed
    output wire                        status_offset_valid,  // pulses when status_offset changes
    output wire [63:0]                 status_adj            // rate correction, ppb * 2^16, signed
);

// check configuration
initial begin
    if (PERIOD_NS_WIDTH + FNS_WIDTH > 40) begin
        $error("Error: PERIOD_NS_WIDTH + FNS_WIDTH must be 40 or less (instance %m)");
        $finish;
    end
end

/*
 * Time of day arithmetic on ptp_clock's 96 bit format,
 * {seconds[47:0], 2'b00, ns[29:0], fns[15:0]}. Differences and offsets are
 * signed 64 bit values in ns * 2^16.
 */

// 10^9 ns in units of 2^-16 ns
localparam [63:0] NS_PER_S_Q16 = 64'd65536000000000;

// a signed number of seconds, -8 to 7, in ns * 2^16
function [63:0] sec_to_q16;
    input [3:0] s;
    begin
        case (s)
            4'd0:  sec_to_q16 = 64'd0;
            4'd1:  sec_to_q16 = NS_PER_S_Q16;
            4'd2:  sec_to_q16 = NS_PER_S_Q16*2;
            4'd3:  sec_to_q16 = NS_PER_S_Q16*3;
            4'd4:  sec_to_q16 = NS_PER_S_Q16*4;
            4'd5:  sec_to_q16 = NS_PER_S_Q16*5;
            4'd6:  sec_to_q16 = NS_PER_S_Q16*6;
            4'd7:  sec_to_q16 = NS_PER_S_Q16*7;
            4'd8:  sec_to_q16 = -(NS_PER_S_Q16*8);
            4'd9:  sec_to_q16 = -(NS_PER_S_Q16*7);
            4'd10: sec_to_q16 = -(NS_PER_S_Q16*6);
            4'd11: sec_to_q16 = -(NS_PER_S_Q16*5);
            4'd12: sec_to_q16 = -(NS_PER_S_Q16*4);
            4'd13: sec_to_q16 = -(NS_PER_S_Q16*3);
            4'd14: sec_to_q16 = -(NS_PER_S_Q16*2);
            default: sec_to_q16 = -NS_PER_S_Q16;
        endcase
    end
endfunction

// a - b; bit 64 is set if the seconds parts are more than 7 apart, in which
// case bits 63:0 are not valid
function [64:0] tod_diff;
    input [95:0] a;
    input [95:0] b;
    reg [48:0] ds;
    reg [47:0] dn;
    begin
        ds = {1'b0, a[95:48]} - {1'b0, b[95:48]};
        dn = {2'b00, a[45:0]} - {2'b00, b[45:0]};
        tod_diff[63:0] = sec_to_q16(ds[3:0]) + {{16{dn[47]}}, dn};
        tod_diff[64] = (ds[48:3] != {46{1'b0}}) && (ds[48:3] != {46{1'b1}});
    end
endfunction

// a + d; bit 96 is set if |d| is 1 s or more, in which case bits 95:0 are not
// valid
function [96:0] tod_add;
    input [95:0] a;
    input [63:0] d;
    reg [63:0] sum;
    reg [47:0] s;
    begin
        sum = {18'd0, a[45:0]} + d;
        s = a[95:48];
        if (sum[63]) begin
            sum = sum + NS_PER_S_Q16;
            s = s - 48'd1;
        end else if (sum >= NS_PER_S_Q16) begin
            sum = sum - NS_PER_S_Q16;
            s = s + 48'd1;
        end
        tod_add[95:0] = {s, 2'b00, sum[45:0]};
        tod_add[96] = d[63] ? ((-d) >= NS_PER_S_Q16) : (d >= NS_PER_S_Q16);
    end
endfunction

// |x| for a signed value
function [63:0] abs64;
    input [63:0] x;
    begin
        abs64 = x[63] ? -x : x;
    end
endfunction

// x limited to [-lim, lim]
function [63:0] clamp64;
    input [63:0] x;
    input [63:0] lim;
    begin
        if (!x[63] && x > lim) begin
            clamp64 = lim;
        end else if (x[63] && (-x) > lim) begin
            clamp64 = -lim;
        end else begin
            clamp64 = x;
        end
    end
endfunction

/*
 * Constants
 */

// nominal clock period, in units of 2^-FNS_WIDTH ns
localparam [63:0] PERIOD_NS_64 = PERIOD_NS;
localparam [63:0] PERIOD_FNS_64 = PERIOD_FNS;
localparam [63:0] PERIOD_FIXED = PERIOD_NS_64 * (64'd1 << FNS_WIDTH) + PERIOD_FNS_64;
// nominal clock period, ns * 2^16
localparam [63:0] PERIOD_Q16 = (PERIOD_FIXED << 16) >> FNS_WIDTH;
// change of period, in units of 2^-FNS_WIDTH ns, per ppb * 2^16 of rate
// change, times 2^32 (for FNS_WIDTH = 32 and 6.4 ns: 27.49 per ppb)
localparam [63:0] PPB_FACTOR = (PERIOD_FIXED * 64'd65536) / 64'd1000000000;

// When stepping, ptp_ts_96 is read in S_STEP_0 and the new time reaches
// ptp_clock 3 cycles later. ptp_clock then outputs the loaded time for 2
// cycles before counting on, as its incrementer runs one cycle ahead of its
// output. The new time is moved on by that many periods, plus the output
// pipeline of ptp_clock.
localparam [63:0] STEP_COMP_Q16 = PERIOD_Q16 * (5 + PTP_CLOCK_PIPELINE);

localparam [63:0] KP_64 = KP;
localparam [63:0] KI_64 = KI;
localparam [63:0] MAX_ADJ_Q16 = 64'd65536 * MAX_PPB;
localparam [63:0] FIRST_STEP_THR_Q16 = 64'd65536 * FIRST_STEP_THRESHOLD_NS;
localparam [63:0] STEP_THR_Q16 = 64'd65536 * STEP_THRESHOLD_NS;
localparam [63:0] LOCK_THR_Q16 = 64'd65536 * LOCK_THRESHOLD_NS;
// offsets are limited to about +/- 16.8 ms before the PI products, which
// keeps them within 64 bits; a larger offset is beyond what the rate limit
// can pull in quickly anyway
localparam [63:0] OFFSET_LIMIT_Q16 = 64'd1 << 40;
// rate measurements need |local elapsed - master elapsed| below this
localparam [63:0] RATE_DIFF_LIMIT_Q16 = 64'd1 << 40;

localparam [1:0]
    MODE_FIRST  = 2'd0,
    MODE_SECOND = 2'd1,
    MODE_TRACK  = 2'd2;

localparam [3:0]
    S_IDLE      = 4'd0,
    S_DIFF      = 4'd1,
    S_DECIDE    = 4'd2,
    S_RATE_MUL  = 4'd3,
    S_RATE_DIV  = 4'd4,
    S_RATE_SET  = 4'd5,
    S_PI_MUL    = 4'd6,
    S_PI_INT    = 4'd7,
    S_PI_SUM    = 4'd8,
    S_APPLY_MUL = 4'd9,
    S_APPLY     = 4'd10,
    S_STEP_0    = 4'd11,
    S_STEP_1    = 4'd12,
    S_STEP_2    = 4'd13;

reg [3:0] state_reg = S_IDLE;
reg [1:0] mode_reg = MODE_FIRST;

// current and previous samples
reg [95:0] local_reg = 96'd0;
reg [95:0] master_reg = 96'd0;
reg [95:0] local_prev_reg = 96'd0;
reg [95:0] master_prev_reg = 96'd0;

reg [63:0] offset_reg = 64'd0;
reg        offset_ovf_reg = 1'b0;
reg [63:0] dl_reg = 64'd0;      // local time elapsed since the previous sample
reg        dl_ovf_reg = 1'b0;
reg [63:0] dm_reg = 64'd0;      // master time elapsed since the previous sample
reg        dm_ovf_reg = 1'b0;

reg [63:0] integ_reg = 64'd0;   // ppb * 2^16
reg [63:0] prop_reg = 64'd0;    // ppb * 2^16
reg [63:0] adj_reg = 64'd0;     // ppb * 2^16
reg [63:0] prod_p_reg = 64'd0;
reg [63:0] prod_i_reg = 64'd0;
reg [63:0] adj_prod_reg = 64'd0;
reg [63:0] period_reg = PERIOD_FIXED;

// rate measurement: |dl - dm| * 10^9 / dl = rate error in ppb
reg        rate_neg_reg = 1'b0;
reg [39:0] rate_diff_reg = 40'd0;
reg [71:0] div_quo_reg = 72'd0;
reg [64:0] div_rem_reg = 65'd0;
reg [63:0] div_den_reg = 64'd0;
reg [6:0]  div_cnt_reg = 7'd0;

reg [63:0] step_diff_reg = 64'd0;
reg        step_ovf_reg = 1'b0;

reg [7:0]  skip_cnt_reg = 8'd0;
reg [15:0] lock_cnt_reg = 16'd0;
reg        locked_reg = 1'b0;

reg [95:0] set_ts_reg = 96'd0;
reg        set_ts_valid_reg = 1'b0;
reg        period_valid_reg = 1'b0;
reg        step_reg = 1'b0;
reg        offset_valid_reg = 1'b0;

/*
 * Datapath
 */
wire [64:0] offset_calc = tod_diff(local_reg, master_reg);
wire [64:0] dl_calc = tod_diff(local_reg, local_prev_reg);
wire [64:0] dm_calc = tod_diff(master_reg, master_prev_reg);
wire [64:0] now_diff_calc = tod_diff(ptp_ts_96, local_reg);
wire [96:0] step_ts_calc = tod_add(master_reg, step_diff_reg);

wire [63:0] offset_abs = abs64(offset_reg);
wire [63:0] offset_limited = clamp64(offset_reg, OFFSET_LIMIT_Q16);

wire [63:0] rate_diff = dl_reg - dm_reg;
wire [63:0] rate_diff_abs = abs64(rate_diff);

// one step of the divide
wire [64:0] div_rem_shift = {div_rem_reg[63:0], div_quo_reg[71]};
wire        div_sub = div_rem_shift >= {1'b0, div_den_reg};

// measured rate error, ppb * 2^16
wire [63:0] rate_meas_abs = (div_quo_reg > MAX_PPB) ? MAX_ADJ_Q16 : {div_quo_reg[47:0], 16'd0};
wire [63:0] rate_meas = rate_neg_reg ? -rate_meas_abs : rate_meas_abs;
wire [63:0] integ_rate_calc = clamp64(integ_reg + rate_meas, MAX_ADJ_Q16);

// PI
wire [63:0] prop_calc = $signed(prod_p_reg) >>> 16;
wire [63:0] integ_inc = $signed(prod_i_reg) >>> 16;
wire [63:0] integ_pi_calc = clamp64(integ_reg + integ_inc, MAX_ADJ_Q16);
wire [63:0] adj_calc = clamp64(prop_reg + integ_reg, MAX_ADJ_Q16);

// period change, units of 2^-FNS_WIDTH ns
wire [63:0] period_delta = $signed(adj_prod_reg) >>> 32;

assign s_sample_ready = state_reg == S_IDLE;

assign ptp_set_ts_96 = set_ts_reg;
assign ptp_set_ts_96_valid = set_ts_valid_reg;
assign ptp_set_period_ns = period_reg[FNS_WIDTH +: PERIOD_NS_WIDTH];
assign ptp_set_period_fns = period_reg[FNS_WIDTH-1:0];
assign ptp_set_period_valid = period_valid_reg;

assign step = step_reg;
assign status_state = mode_reg;
assign status_locked = locked_reg;
assign status_offset = offset_reg;
assign status_offset_valid = offset_valid_reg;
assign status_adj = adj_reg;

always @(posedge clk) begin
    set_ts_valid_reg <= 1'b0;
    period_valid_reg <= 1'b0;
    step_reg <= 1'b0;
    offset_valid_reg <= 1'b0;

    case (state_reg)
        S_IDLE: begin
            if (s_sample_valid) begin
                local_reg <= s_sample_local;
                master_reg <= s_sample_master;
                state_reg <= S_DIFF;
            end
        end
        S_DIFF: begin
            {offset_ovf_reg, offset_reg} <= offset_calc;
            {dl_ovf_reg, dl_reg} <= dl_calc;
            {dm_ovf_reg, dm_reg} <= dm_calc;
            offset_valid_reg <= !offset_calc[64];
            state_reg <= S_DECIDE;
        end
        S_DECIDE: begin
            state_reg <= S_IDLE;
            if (skip_cnt_reg != 8'd0) begin
                skip_cnt_reg <= skip_cnt_reg - 8'd1;
            end else if (mode_reg == MODE_FIRST) begin
                local_prev_reg <= local_reg;
                master_prev_reg <= master_reg;
                mode_reg <= MODE_SECOND;
            end else if (mode_reg == MODE_SECOND) begin
                if (dl_ovf_reg || dm_ovf_reg || dl_reg[63] || dl_reg == 64'd0 ||
                        rate_diff_abs >= RATE_DIFF_LIMIT_Q16) begin
                    // the two samples can't give a rate; start again from this one
                    local_prev_reg <= local_reg;
                    master_prev_reg <= master_reg;
                end else begin
                    // positive: the local clock ran fast
                    rate_neg_reg <= rate_diff[63];
                    rate_diff_reg <= rate_diff_abs[39:0];
                    state_reg <= S_RATE_MUL;
                end
            end else begin
                if (offset_ovf_reg || (STEP_THRESHOLD_NS != 0 && offset_abs > STEP_THR_Q16)) begin
                    state_reg <= S_STEP_0;
                end else begin
                    state_reg <= S_PI_MUL;
                end
            end
        end
        S_RATE_MUL: begin
            div_quo_reg <= {32'd0, rate_diff_reg} * 72'd1000000000;
            div_rem_reg <= 65'd0;
            div_den_reg <= dl_reg;
            div_cnt_reg <= 7'd72;
            state_reg <= S_RATE_DIV;
        end
        S_RATE_DIV: begin
            div_rem_reg <= div_sub ? div_rem_shift - {1'b0, div_den_reg} : div_rem_shift;
            div_quo_reg <= {div_quo_reg[70:0], div_sub};
            div_cnt_reg <= div_cnt_reg - 7'd1;
            if (div_cnt_reg == 7'd1) begin
                state_reg <= S_RATE_SET;
            end
        end
        S_RATE_SET: begin
            integ_reg <= integ_rate_calc;
            adj_reg <= integ_rate_calc;
            mode_reg <= MODE_TRACK;
            lock_cnt_reg <= 16'd0;
            locked_reg <= 1'b0;
            if (offset_ovf_reg || offset_abs > FIRST_STEP_THR_Q16) begin
                state_reg <= S_STEP_0;
            end else begin
                state_reg <= S_APPLY_MUL;
            end
        end
        S_PI_MUL: begin
            prod_p_reg <= offset_limited * KP_64;
            prod_i_reg <= offset_limited * KI_64;
            state_reg <= S_PI_INT;
        end
        S_PI_INT: begin
            prop_reg <= prop_calc;
            integ_reg <= integ_pi_calc;
            state_reg <= S_PI_SUM;
        end
        S_PI_SUM: begin
            adj_reg <= adj_calc;
            if (offset_abs <= LOCK_THR_Q16) begin
                if (lock_cnt_reg + 16'd1 >= LOCK_COUNT) begin
                    locked_reg <= 1'b1;
                end else begin
                    lock_cnt_reg <= lock_cnt_reg + 16'd1;
                end
            end else begin
                lock_cnt_reg <= 16'd0;
                locked_reg <= 1'b0;
            end
            state_reg <= S_APPLY_MUL;
        end
        S_APPLY_MUL: begin
            adj_prod_reg <= adj_reg * PPB_FACTOR;
            state_reg <= S_APPLY;
        end
        S_APPLY: begin
            period_reg <= PERIOD_FIXED - period_delta;
            period_valid_reg <= 1'b1;
            state_reg <= S_IDLE;
        end
        S_STEP_0: begin
            // time since the sample, read from ptp_ts_96 in this cycle
            step_diff_reg <= now_diff_calc[63:0];
            step_ovf_reg <= now_diff_calc[64];
            state_reg <= S_STEP_1;
        end
        S_STEP_1: begin
            step_diff_reg <= step_diff_reg + STEP_COMP_Q16;
            state_reg <= S_STEP_2;
        end
        S_STEP_2: begin
            // new time = master time of the sample + time since the sample
            if (!step_ovf_reg && !step_ts_calc[96]) begin
                set_ts_reg <= step_ts_calc[95:0];
                set_ts_valid_reg <= 1'b1;
                step_reg <= 1'b1;
                skip_cnt_reg <= STEP_SKIP_SAMPLES;
                mode_reg <= MODE_FIRST;
                lock_cnt_reg <= 16'd0;
                locked_reg <= 1'b0;
            end
            // otherwise the sample is more than a second old; the next sample
            // steps instead
            state_reg <= S_APPLY_MUL;
        end
        default: begin
            state_reg <= S_IDLE;
        end
    endcase

    if (restart) begin
        state_reg <= S_IDLE;
        mode_reg <= MODE_FIRST;
        skip_cnt_reg <= 8'd0;
        lock_cnt_reg <= 16'd0;
        locked_reg <= 1'b0;
    end

    if (rst) begin
        state_reg <= S_IDLE;
        mode_reg <= MODE_FIRST;
        integ_reg <= 64'd0;
        prop_reg <= 64'd0;
        adj_reg <= 64'd0;
        period_reg <= PERIOD_FIXED;
        skip_cnt_reg <= 8'd0;
        lock_cnt_reg <= 16'd0;
        locked_reg <= 1'b0;
        set_ts_valid_reg <= 1'b0;
        period_valid_reg <= 1'b0;
        step_reg <= 1'b0;
        offset_valid_reg <= 1'b0;
    end
end

endmodule

`resetall
