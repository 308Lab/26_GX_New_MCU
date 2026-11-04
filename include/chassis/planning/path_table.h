/**
  ******************************************************************************
  * @file    chassis/planning/path_table.h
  * @brief   内置路径表声明。表体以 const 存于 Flash，不占 RAM。
  ******************************************************************************
  */

#ifndef CHASSIS_PLANNING_PATH_TABLE_H
#define CHASSIS_PLANNING_PATH_TABLE_H

#include "chassis/planning/path.h"

extern const Path_Table_t *const g_chassis_path_table[];
extern const uint16_t            g_chassis_path_count;

#endif /* CHASSIS_PLANNING_PATH_TABLE_H */
