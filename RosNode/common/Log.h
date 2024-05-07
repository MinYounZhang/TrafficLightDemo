#pragma once
#include <string>
#include <sstream>
#include "ErrorCode.h"

// 写入日志
void writeLog(const std::string &info, int logLevel);

namespace common_log
{
	enum Log_Level
	{
		Log_Level_Verbose = 0,		// 详细的日志级
		Log_Level_Debug = 10,		// debug级别的日
		Log_Level_Info = 20,		// info级别的打
        Log_Level_Warn = 30,		// warn级别的打
        Log_Level_Error = 40,		// error 级别的打
        LOG_Level_Fatal = 50,       // 致命的错误
	};

	// 设置日志的配置文件，如果不设置，则使用默认配
    void setLogConfigFile(const std::string fileConfig);

	// 设置日志级别
    void setLogLevel(Log_Level logLevel);

    // 获取日志级别
    Log_Level getLogLevel();

    // 写日志(forceWrite如果为true,无论日志级别，强制写入)
    void writeLog(const std::string &info, Log_Level logLevel = Log_Level_Info);
}
