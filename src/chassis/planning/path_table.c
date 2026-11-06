/**
  ******************************************************************************
  * @file    chassis/planning/path_table.c
  * @brief   底盘内置路径表（全部 const，驻留 Flash，不占 RAM）
  *
  * 这份文件只回答一个问题：车该按什么顺序走哪些段。
  * 每张表 = 一条可执行的段序列，由 Chassis_Path_SelectById() / SelectByName()
  * 选中后 Chassis_Path_Start() 开跑，段调度与超时判断都在 path.c 里。
  *
  * ⚠ 表号是硬契约：mission.c 用 SelectById(0..4) 直接取表。
  *    往数组末尾追加新表是安全的；插到中间或调换顺序会打乱全部流程。
  *
  * ── 怎么改 ────────────────────────────────────────────────────────────────
  *   直行      SEG_FWD(毫米, 速度)              例：SEG_FWD(625.0f, 200.0f)
  *   后退      SEG_BACK(毫米, 速度)
  *   左/右弧   SEG_ARC_L(角度, 半径, 速度)      两个都是正数，左/右看宏名
  *   原地补正  SPIN_ALIGN()                     紧跟在弧段后，收掉提前制动造成的欠转
  *   任务点    SEG_TASK('B', 超时毫秒)          停车等外部 NotifyTaskDone
  *   重标定    SEG_CALIB()                      停车重标陀螺零偏
  *
  * ── 三个容易踩的点 ────────────────────────────────────────────────────────
  *   1) 弧段的转向只由 dir 的正负表达（左/右），线速度恒为正。
  *      这是审计 H12 之后的行为：老代码靠负速度表达后退画弧，会被控制层的
  *      最小速度钳位翻正，右弧因此拿不到减速规划。别再按老注释改这条。
  *   2) 表里的距离和转角仍是上车实测前的参考值，改完必须实车复验。
  *   3) 每张表末尾的任务点超时是「兜底」而不是「等待时长」，别调得比
  *      实际动作还短，否则任务做到一半就被判 PATH_EV_ERROR。
  *
  * ── Path_Seg_t 字段顺序（宏就是按这个顺序拼出来的，手写时照此对齐）──────
  *     1 type        5 radius_mm      9  brake
  *     2 dir         6 v_mmps        10  timeout_ms
  *     3 dist_mm     7 a_mmps2       11  point_id
  *     4 angle_deg   8 dwell_ms
  *   宏里没用到的字段一律填 0 / 0U，别顺手留下上一段的值。
  ******************************************************************************
  */

#include "chassis/planning/path_table.h"

/* 段构造宏：把 11 个位置参数收在宏里，改表时只需要关心真正有意义的几个值。
   全部展开成同一种初始化器，不影响 Flash 布局，也不改变运行行为。 */

#define SEG_FWD(mm, v)                                                        \
    { PATH_STRAIGHT, +1, (mm), 0.0f, 0.0f, (v), 0.0f, 0U, 1U, 0U, 0U }

#define SEG_BACK(mm, v)                                                       \
    { PATH_STRAIGHT, -1, (mm), 0.0f, 0.0f, (v), 0.0f, 0U, 1U, 0U, 0U }

#define SEG_ARC_L(deg, r, v)                                                  \
    { PATH_ARC, +1, 0.0f, (deg), (r), (v), 0.0f, 0U, 1U, 0U, 0U }

#define SEG_ARC_R(deg, r, v)                                                  \
    { PATH_ARC, -1, 0.0f, (deg), (r), (v), 0.0f, 0U, 1U, 0U, 0U }

/* SPIN(0°) = 原地闭环回计划航向，用于补正 ARC 提前制动造成的欠转。
   转多少不由这里给：控制层拿「计划航向 − 实际航向」做闭环，所以航向已到位
   时这一段会立刻完成，没到位就补上去。 */
#define SPIN_ALIGN() \
    { PATH_SPIN, 0, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0U, 1U, 0U, 0U }

#define SEG_CALIB() \
    { PATH_CALIB, 0, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0U, 1U, 0U, 0U }

/* 任务点：停车 → 发 PATH_EV_ARRIVE_POINT(id) → 等 NotifyTaskDone(id)。
   timeout_ms 是这个等待的上限，到点没等到就报 PATH_EV_ERROR 并停车。 */
#define SEG_TASK(id_ch, timeout_ms)                                           \
    { PATH_TASK, 0, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0U, 1U, (timeout_ms),       \
      (uint8_t)(id_ch) }

/* ══════════════════════════════════════════════════════════════════════════
   表 0：起点 → A
   谁在用：按键 1、按键 3 的开头，正式流程的第一段。
   走法：直行 625 到 A 点，停车等 A 点任务（扫码/抓取）做完。
   注意：这里等的是「任务完成」，不是「到达」，所以超时给到 15 s；
         A 点动作变长时记得同步放宽。
   ══════════════════════════════════════════════════════════════════════════ */
static const Path_Seg_t s_start_to_A_segs[] =
{
    SEG_FWD(625.0f, 200.0f),          /* 起点 → A 点 */
    SEG_TASK('A', 15000U),            /* A 点任务，最长等 15 s */
};

/* ══════════════════════════════════════════════════════════════════════════
   表 1：A → B
   谁在用：按键 2；正式流程与演示流程都用这一条。
   走法：A 点做完先重标陀螺零偏，退出来重新对线，
         然后是一串「直行—转弯」的走廊，最后停在任务区中间。
   账目：前进段合计 1440 mm（110 + 360 + 260 + 350 + 360），
         另有一段 300 mm 后退，净推进 1140 mm；
         中途 5 个 90° 弧（左3 右2），每个弧后都跟一次 SPIN_ALIGN 补航向。
   ══════════════════════════════════════════════════════════════════════════ */
static const Path_Seg_t s_A_to_B_segs[] =
{
    SEG_CALIB(),                      /* 出发前重标零偏，消掉 A 点停留期间的漂移 */
    SEG_BACK(300.0f, 200.0f),         /* 先退出 A 点通道 */

    SEG_ARC_L(90.0f, 200.0f, 200.0f), /* 左弧转入 B 区走廊 */
    SPIN_ALIGN(),

    SEG_FWD(110.0f, 200.0f),          /* 第一段短直行 */
    SEG_ARC_R(90.0f, 200.0f, 150.0f), /* 右弧 */
    SPIN_ALIGN(),

    SEG_FWD(350.0f, 150.0f),          /* 过减速带，降到 150 */
    SEG_ARC_L(90.0f, 200.0f, 200.0f), /* 左弧 */
    SPIN_ALIGN(),

    SEG_FWD(255.0f, 200.0f),          /* 中段直行 */
    SEG_ARC_R(90.0f, 200.0f, 200.0f), /* 右弧 */
    SPIN_ALIGN(),

    SEG_FWD(350.0f, 200.0f),          /* 接近任务区 */
    SEG_ARC_R(90.0f, 200.0f, 200.0f), /* 最后一个右弧，摆正朝向 */
    SPIN_ALIGN(),

    SEG_FWD(355.0f, 200.0f),          /* 走到任务区中间停车 */
    SEG_TASK('B', 30000U),            /* B 点任务：扫码 → 中位线 → 名次 → 抓取 */
};

/* ══════════════════════════════════════════════════════════════════════════
   表 2：B → C
   谁在用：正式流程 B 段结束后由 mission.c 选表，没有独立按键。
   走法：一条长直道穿过两个房间，最后停在 C 点任务区中间。
   账目：1400 + 300 = 1700 mm。
   ══════════════════════════════════════════════════════════════════════════ */
static const Path_Seg_t s_B_to_C_segs[] =
{
    SEG_FWD(1400.0f, 200.0f),         /* B 区 → C 区主通道 */
    SEG_FWD(300.0f, 200.0f),          /* C 区任务区中间 */
    SEG_TASK('C', 30000U),
};

/* ══════════════════════════════════════════════════════════════════════════
   表 3：C → D
   谁在用：按键 7；正式流程 C 段结束后自动接上。
   走法：直行一小段 → 右转 90° → 穿过一段直道 → 停在 D 点任务区中间。
   账目：305 + 320 + 300 = 925 mm，转角 90°（右）。
   ══════════════════════════════════════════════════════════════════════════ */
static const Path_Seg_t s_C_to_D_segs[] =
{
    SEG_FWD(335.0f, 200.0f),
    SEG_ARC_R(90.0f, 200.0f, 200.0f), /* 右转进 D 区走廊 */
    SPIN_ALIGN(),
    SEG_FWD(320.0f, 200.0f),
    SEG_FWD(300.0f, 200.0f),          /* D 点任务区中间 */
    SEG_TASK('D', 30000U),
};

/* ══════════════════════════════════════════════════════════════════════════
   表 4：D → 终点
   谁在用：按键 8；正式流程最后一段。
   走法：一条 1700 mm 直道冲线，没有转弯也没有任务点。
   ══════════════════════════════════════════════════════════════════════════ */
static const Path_Seg_t s_D_to_end_segs[] =
{
    SEG_FWD(1700.0f, 200.0f),
};

/* 表描述体：段数由 sizeof 自动算，加行减行都不用手改计数 */
#define PATH_TABLE_ENTRY(name_lit, segs_arr)                                  \
    { name_lit, segs_arr,                                                     \
      (uint16_t)(sizeof(segs_arr) / sizeof((segs_arr)[0])) }

static const Path_Table_t s_path_start_to_A =
    PATH_TABLE_ENTRY("to_A", s_start_to_A_segs);
static const Path_Table_t s_path_A_to_B =
    PATH_TABLE_ENTRY("A_B", s_A_to_B_segs);
static const Path_Table_t s_path_B_to_C =
    PATH_TABLE_ENTRY("B_C", s_B_to_C_segs);
static const Path_Table_t s_path_C_to_D =
    PATH_TABLE_ENTRY("C_D", s_C_to_D_segs);
static const Path_Table_t s_path_D_to_end =
    PATH_TABLE_ENTRY("D_end", s_D_to_end_segs);

/* ⚠ 顺序即表号：mission.c 的 SelectById(0..4) 依赖这个顺序，只能往末尾追加 */
 const Path_Table_t *const g_chassis_path_table[] =
{
    &s_path_start_to_A,
    &s_path_A_to_B,
    &s_path_B_to_C,
    &s_path_C_to_D,
    &s_path_D_to_end
};

const uint16_t g_chassis_path_count =
    (uint16_t)(sizeof(g_chassis_path_table) / sizeof(g_chassis_path_table[0]));
