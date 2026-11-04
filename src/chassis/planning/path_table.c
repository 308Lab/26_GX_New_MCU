/**
  ******************************************************************************
  * @file    chassis/planning/path_table.c
  * @brief   内置路径表。全部为 const，驻留 Flash，不占 RAM。
  *
  * 字段顺序：type, dir, dist_mm, angle_deg, radius_mm, v_mmps, a_mmps2,
  *           dwell_ms, brake, timeout_ms, point_id
  *
  * 当前距离、扫描段长度、任务超时均为占位值，待地图实测后替换。
  ******************************************************************************
  */

#include "chassis/planning/path_table.h"

/* SPIN(0°) = 原地闭环回计划航向，用于补正 ARC 提前制动造成的欠转 */
#define SPIN_ALIGN() \
    { PATH_SPIN, 0, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0U, 1U, 0U, 0U }

/* 起点 -> A：分段测试版
   直行 280 → 右转 90° → 前进 35 → 慢速左转 90° → 慢行 350 → 左转 90° → 直行 315 到 A */
static const Path_Seg_t s_start_to_A_segs[] =
{
    { PATH_STRAIGHT, +1, 280.0f,  0.0f,   0.0f, 200.0f, 0.0f,   0U, 1U, 0U, 0U },
    { PATH_ARC,      -1,   0.0f, 90.0f, 200.0f, 200.0f, 0.0f,   0U, 1U, 0U, 0U },
    SPIN_ALIGN(),
    { PATH_STRAIGHT, +1,  55.0f,  0.0f,   0.0f, 200.0f, 0.0f,   0U, 1U, 0U, 0U },
    { PATH_ARC,      +1,   0.0f, 90.0f, 200.0f, 120.0f, 0.0f,   0U, 1U, 0U, 0U },
    SPIN_ALIGN(),
    { PATH_STRAIGHT, +1, 370.0f,  0.0f,   0.0f, 150.0f, 0.0f,   0U, 1U, 0U, 0U },
    { PATH_ARC,      +1,   0.0f, 90.0f, 200.0f, 200.0f, 0.0f,   0U, 1U, 0U, 0U },
    SPIN_ALIGN(),
    { PATH_STRAIGHT, +1, 315.0f,  0.0f,   0.0f, 200.0f, 0.0f,   0U, 1U, 0U, 0U },
    /* A 点：停车发 ARRIVE('A')，等待任务完成通知 */
    { PATH_TASK,      0,   0.0f,  0.0f,   0.0f,   0.0f, 0.0f,   0U, 1U, 15000U, (uint8_t)'A' },
};

/* 表 1：A -> B（按键 3 触发）
   直行 1350 → 右转 90° → 直行 380 → 右转 90° → 慢行扫描 600 */
static const Path_Seg_t s_A_to_B_segs[] =
{
    /* A 点任务完成后，出发前重新标定零偏 */
    { PATH_CALIB,     0,   0.0f,  0.0f,   0.0f,   0.0f, 0.0f,   0U, 1U, 0U, 0U },
    { PATH_STRAIGHT, +1, 1300.0f, 0.0f,   0.0f, 200.0f, 0.0f,   0U, 1U, 0U, 0U },
    { PATH_ARC,      -1,   0.0f, 90.0f, 200.0f, 200.0f, 0.0f,   0U, 1U, 0U, 0U },
    SPIN_ALIGN(),
    { PATH_STRAIGHT, +1, 340.0f,  0.0f,   0.0f, 200.0f, 0.0f,   0U, 1U, 0U, 0U },
    { PATH_ARC,      -1,   0.0f, 90.0f, 200.0f, 200.0f, 0.0f,   0U, 1U, 0U, 0U },
    SPIN_ALIGN(),
    /* 识别扫描段：慢行 700，走完后自动倒回 350 到段中心（演示模式只取中值） */
    { PATH_SCAN,     +1, 700.0f,  0.0f,   0.0f, 100.0f, 0.0f, 300U, 1U, 0U, (uint8_t)'B' },
};

/* 表 2：B -> C（按键 7 触发）
   直行 1400 → 慢行扫描 700 → 倒回 350 */
static const Path_Seg_t s_B_to_C_segs[] =
{
    { PATH_STRAIGHT, +1, 1400.0f, 0.0f,   0.0f, 200.0f, 0.0f,   0U, 1U, 0U, 0U },
    { PATH_SCAN,     +1,  700.0f, 0.0f,   0.0f, 100.0f, 0.0f, 300U, 1U, 0U, (uint8_t)'C' },
};

/* 表 3：C -> D（按键 7 触发）
   直行 360 → 右转 90° → 直行 300 → 慢行扫描 500 → 倒回 250 */
static const Path_Seg_t s_C_to_D_segs[] =
{
    { PATH_STRAIGHT, +1, 305.0f,  0.0f,   0.0f, 200.0f, 0.0f,   0U, 1U, 0U, 0U },
    { PATH_ARC,      -1,   0.0f, 90.0f, 200.0f, 200.0f, 0.0f,   0U, 1U, 0U, 0U },
    SPIN_ALIGN(),
    { PATH_STRAIGHT, +1, 300.0f,  0.0f,   0.0f, 200.0f, 0.0f,   0U, 1U, 0U, 0U },
    { PATH_SCAN,     +1, 600.0f,  0.0f,   0.0f, 100.0f, 0.0f, 300U, 1U, 0U, (uint8_t)'D' },
};

/* 表 4：D -> 终点（按键 8 触发）
   直行 1700 */
static const Path_Seg_t s_D_to_end_segs[] =
{
    { PATH_STRAIGHT, +1, 1730.0f, 0.0f,   0.0f, 200.0f, 0.0f,   0U, 1U, 0U, 0U },
};

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
