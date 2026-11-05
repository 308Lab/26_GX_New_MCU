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

/* 起点 -> A：直走一段后停车扫码（距离为参考值，待实测） */
static const Path_Seg_t s_start_to_A_segs[] =
{
    { PATH_STRAIGHT, +1, 625.0f, 0.0f,   0.0f, 200.0f, 0.0f,   0U, 1U, 0U, 0U },
    /* A 点：停车发 ARRIVE('A')，等待任务完成通知 */
    { PATH_TASK,      0,   0.0f,  0.0f,   0.0f,   0.0f, 0.0f,   0U, 1U, 15000U, (uint8_t)'A' },
};

/* 表 1：A -> B（按键 2 / 正式与演示流程使用）
   后退 300 → 左弧 90° → 直行 130 → 右弧 90° → 直行 340@150（减速带）
   → 左弧 90° → 直行 → 右弧 90° → 直行 → 右弧 90° → 直行
   → 直行到任务区中间停车
   停车点 = 原扫描段 700 的中点（350），任务完成由 ARRIVE 通知解除。
   距离参考原表注释（直行合计约 1680 mm），待实测。 */
static const Path_Seg_t s_A_to_B_segs[] =
{
    /* A 点任务完成后，出发前重新标定零偏 */
    { PATH_CALIB,     0,   0.0f,  0.0f,   0.0f,   0.0f, 0.0f,   0U, 1U, 0U, 0U },
    /* 先退出一段，再左弧转入 B 区通道 */
    { PATH_STRAIGHT, -1, 300.0f,  0.0f,   0.0f, 200.0f, 0.0f,   0U, 1U, 0U, 0U },
    { PATH_ARC,      +1,   0.0f, 90.0f, 200.0f, 200.0f, 0.0f,   0U, 1U, 0U, 0U },
    SPIN_ALIGN(),
    { PATH_STRAIGHT, +1,  110.0f,  0.0f,   0.0f, 200.0f, 0.0f,   0U, 1U, 0U, 0U },
    { PATH_ARC,      -1,   0.0f, 90.0f, 200.0f, 200.0f, 0.0f,   0U, 1U, 0U, 0U },
    SPIN_ALIGN(),
    /* 过减速带 */
    { PATH_STRAIGHT, +1, 360.0f,  0.0f,   0.0f, 150.0f, 0.0f,   0U, 1U, 0U, 0U },
    { PATH_ARC,      +1,   0.0f, 90.0f, 200.0f, 200.0f, 0.0f,   0U, 1U, 0U, 0U },
    SPIN_ALIGN(),
    { PATH_STRAIGHT, +1, 260.0f,  0.0f,   0.0f, 200.0f, 0.0f,   0U, 1U, 0U, 0U },
    { PATH_ARC,      -1,   0.0f, 90.0f, 200.0f, 200.0f, 0.0f,   0U, 1U, 0U, 0U },
    SPIN_ALIGN(),
    { PATH_STRAIGHT, +1, 350.0f,  0.0f,   0.0f, 200.0f, 0.0f,   0U, 1U, 0U, 0U },
    { PATH_ARC,      -1,   0.0f, 90.0f, 200.0f, 200.0f, 0.0f,   0U, 1U, 0U, 0U },
    SPIN_ALIGN(),
    /* 任务区中间：原扫描段 700 的中点，停车后展开机械臂并请求视觉 */
    { PATH_STRAIGHT, +1, 360.0f,  0.0f,   0.0f, 200.0f, 0.0f,   0U, 1U, 0U, 0U },
    { PATH_TASK,      0,   0.0f,  0.0f,   0.0f,   0.0f, 0.0f,   0U, 1U, 30000U, (uint8_t)'B' },
};

/* 表 2：B -> C（正式与演示流程使用）
   直行 1400 → 直行 350 到任务区中间停车 */
static const Path_Seg_t s_B_to_C_segs[] =
{
    { PATH_STRAIGHT, +1, 1400.0f, 0.0f,   0.0f, 200.0f, 0.0f,   0U, 1U, 0U, 0U },
    { PATH_STRAIGHT, +1,  300.0f, 0.0f,   0.0f, 200.0f, 0.0f,   0U, 1U, 0U, 0U },
    { PATH_TASK,      0,   0.0f,  0.0f,   0.0f,   0.0f, 0.0f,   0U, 1U, 30000U, (uint8_t)'C' },
};

/* 表 3：C -> D（按键 7 / 正式与演示流程使用）
   直行 305 → 右转 90° → 直行 300 → 直行 300 到任务区中间停车 */
static const Path_Seg_t s_C_to_D_segs[] =
{
    { PATH_STRAIGHT, +1, 305.0f,  0.0f,   0.0f, 200.0f, 0.0f,   0U, 1U, 0U, 0U },
    { PATH_ARC,      -1,   0.0f, 90.0f, 200.0f, 200.0f, 0.0f,   0U, 1U, 0U, 0U },
    SPIN_ALIGN(),
    { PATH_STRAIGHT, +1, 320.0f,  0.0f,   0.0f, 200.0f, 0.0f,   0U, 1U, 0U, 0U },
    { PATH_STRAIGHT, +1, 300.0f,  0.0f,   0.0f, 200.0f, 0.0f,   0U, 1U, 0U, 0U },
    { PATH_TASK,      0,   0.0f,  0.0f,   0.0f,   0.0f, 0.0f,   0U, 1U, 30000U, (uint8_t)'D' },
};

/* 表 4：D -> 终点（按键 8 触发）
   直行 1700 */
static const Path_Seg_t s_D_to_end_segs[] =
{
    { PATH_STRAIGHT, +1, 1700.0f, 0.0f,   0.0f, 200.0f, 0.0f,   0U, 1U, 0U, 0U },
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
