/*
    Logging Class
    2/15/20

    Nicholas Seidl
    Modified by Scott Coursin
    
*/

#include "Logger.h"

const std::vector<char> c_vecLevels
{
      'D'
    , 'I'
    , 'W'
    , 'E'
};

Logger::Logger(const char *path, bool console_echo)
    : m_fd(nullptr)
    , m_path(path)
    , m_console_echo(console_echo)
{
    m_formattedIntData.reserve(500);
    m_formattedDoubleData.reserve(500);
}

void Logger::openLog()
{
    printf("Opening log %s\n", m_path.c_str());
    if (m_fd != nullptr)
    {
        printf("Log was open, closing\n");
        closeLog();
    }

    m_fd = fopen(m_path.c_str(), "a");
    // CSV header for free form text logging
    if (m_fd != nullptr)
        fprintf(m_fd, "Timestamp,Level,Function,Line,Message\n");

    if (m_console_echo)
        printf("Timestamp,Level,Function,Line,Message\n");
}

Logger::~Logger()
{
    closeLog();
}

void Logger::closeLog()
{
    if (m_fd != nullptr)
    {
        printf("Closing log\n");
        fclose(m_fd);
        m_fd = nullptr;
    }
    else
    {
        printf("No log was open\n");
    }
}

void Logger::logMsg(ELogLevel level, const char* func, const int line, const char* msg, const char* msg2 /* = nullptr */)
{
    if (m_fd == nullptr)
    {
        openLog();
    }

    if (m_fd != nullptr)
    {
        float timestamp = m_timer.GetFPGATimestamp();
        if (msg2 == nullptr)
        {
	        fprintf(m_fd, "%.6f,%c,%s,%d,%s\n", timestamp, c_vecLevels[level], func, line, msg);
        }
        else
        {
            fprintf(m_fd, "%.6f,%c,%s,%d,%s,%s\n", timestamp, c_vecLevels[level], func, line, msg, msg2);
        }
        
        if (m_console_echo)
        {
            if (msg2 == nullptr)
            {
	            printf("%.6f,%c,%s,%d,%s\n", timestamp, c_vecLevels[level], func, line, msg);
            }
            else
            {
                printf("%.6f,%c,%s,%d,%s,%s\n", timestamp, c_vecLevels[level], func, line, msg, msg2);
            }
        }
    }
}

void Logger::logData(const char* func, const int line, const vector<double*>& data)
{
    formatData(data);
    logMsg(eInfo, func, line, m_formattedDoubleData.c_str());
}

void Logger::logData(const char* func, const int line, const vector<int*>& data)
{
    formatData(data);
    logMsg(eInfo, func, line, m_formattedIntData.c_str());
}

void Logger::logData(const char* func, const int line, const vector<int*>& dataInt, const vector<double*>& dataDouble)
{
    formatData(dataInt);
    formatData(dataDouble);
    logMsg(eInfo, func, line, m_formattedIntData.c_str(), m_formattedDoubleData.c_str());
}

void Logger::formatData(const vector<double*>& data)
{
	//									   123456789
    constexpr int c_charsPerDouble = 9;	// -123.000, most will be small but allow for some to be larger
    const size_t c_numDataPts = data.size();
    const int c_totalSz = c_charsPerDouble * c_numDataPts - 1;  // -1 to truncate the terminating null
    m_formattedDoubleData.resize(c_totalSz);
    char* out = const_cast<char*>(m_formattedDoubleData.c_str());

    for (size_t i = 0; i < c_numDataPts; i++)
    {
        //out += sprintf(out, "%.3f%s", *data[i], i == c_numDataPts - 1 ? "" : ",");
        out += sprintf(out, "%.3f%c", *data[i], i < c_numDataPts - 1 ? ',' : ' ');
 		if (out - m_formattedDoubleData.c_str() >= c_totalSz)
		{
			break;
		}
    }
}

void Logger::formatData(const vector<double>& data)
{
	//									   123456789
    constexpr int c_charsPerDouble = 9;	// -123.000, most will be small but allow for some to be larger
    const size_t c_numDataPts = data.size();
    const int c_totalSz = c_charsPerDouble * c_numDataPts - 1;  // -1 to truncate the terminating null
    m_formattedDoubleData.resize(c_totalSz);
    char* out = const_cast<char*>(m_formattedDoubleData.c_str());

    for (size_t i = 0; i < c_numDataPts; i++)
    {
        //out += sprintf(out, "%.3f%s", data[i], i == c_numDataPts - 1 ? "" : ",");
        out += sprintf(out, "%.3f%c", data[i], i < c_numDataPts - 1 ? ',' : ' ');
 		if (out - m_formattedDoubleData.c_str() >= c_totalSz)
		{
			break;
		}
    }
}

void Logger::formatData(const vector<int*>& data)
{
    //									   1234567
    constexpr int c_charsPerInt = 7;	// _ _ _0,
    const size_t c_numDataPts = data.size();
    const int c_totalSz = c_charsPerInt * c_numDataPts - 1;  // -1 to truncate the terminating null
    m_formattedIntData.resize(c_totalSz);
    char* out = const_cast<char*>(m_formattedIntData.c_str());

    for (size_t i = 0; i < c_numDataPts; i++)
    {
        //out += sprintf(out, "%6d%s", *data[i], i == c_numDataPts - 1 ? "" : ",");
        out += sprintf(out, "%6d%c", *data[i], i < c_numDataPts - 1 ? ',' : ' ');
        if (out - m_formattedIntData.c_str() >= c_totalSz)
        {
            break;
        }
    }
}

void Logger::formatData(const vector<int>& data)
{
    //									   1234567
    constexpr int c_charsPerInt = 7;	// _ _ _0,
    const size_t c_numDataPts = data.size();
    const int c_totalSz = c_charsPerInt * c_numDataPts - 1;  // -1 to truncate the terminating null
    m_formattedIntData.resize(c_totalSz);
    char* out = const_cast<char*>(m_formattedIntData.c_str());

    for (size_t i = 0; i < c_numDataPts; i++)
    {
        //out += sprintf(out, "%6d%s", data[i], i == c_numDataPts - 1 ? "" : ",");
        out += sprintf(out, "%6d%c", data[i], i < c_numDataPts - 1 ? ',' : ' ');
        if (out - m_formattedIntData.c_str() >= c_totalSz)
        {
            break;
        }
    }
}
