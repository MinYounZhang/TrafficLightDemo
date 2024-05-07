#pragma once
#include "Base.h"

/*
 * 错误码枚举
 */

// 告警ID
enum WarnCode
{
    WARN_XXX1 = WARN_LOG_ID+1,
};

// 错误ID
enum ErrorCode
{
    ERROR_NOT_FIND_PARAM = ERROR_LOG_ID+1  // 参数错误
};

// 致命错误id
enum FatalCode
{
    FATAL_INVALID_CONFIG = FATAL_LOG_ID+1,      // 配置信息异常
    FATAL_CLOUD_EMPTY,    // 点云为空
    FATAL_ROI_EMPTY,       //ROI内点云为空
    FATAL_GROUND_ESTIMATE //地面分割异常
};
