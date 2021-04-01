/*
    Logging Class
    2/15/20

    Nicholas Seidl
    Modified by Scott Coursin
    
*/

#include "Logger.h"

#include <dirent.h>

const std::vector<char> c_vecLevels
{
      'D'
    , 'I'
    , 'W'
    , 'E'
};

Logger::Logger(bool console_echo)
    : m_fd(nullptr)
    , m_console_echo(console_echo)
{
    m_formattedIntData.reserve(500);
    m_formattedDoubleData.reserve(500);
    m_formattedData.reserve(1000);
}

void Logger::openLog()
{
    std::string path = c_logFileBasePath + FindNextLogFileNumber() + c_logFileExtension;
    printf("Opening log %s\n", path.c_str());
    if (m_fd != nullptr)
    {
        printf("Log was open, closing\n");
        closeLog();
    }

    m_fd = fopen(path.c_str(), "a");
    // CSV header for free form text logging
    if (m_fd != nullptr)
        fprintf(m_fd, "Timestamp,Header,Source,Level,Message\n");

    if (m_console_echo)
        printf("Timestamp,Header,Source,Level,Message\n");
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

std::string Logger::FindNextLogFileNumber()
{
    int nextLogFileNumber = 0;
    struct dirent *dir;
    DIR* d = opendir("/tmp");
    //printf("Existing log files\n");
    if (d)
    {
        while ((dir = readdir(d)) != NULL)
        {
            //printf("%s\n", dir->d_name);
            std::string filename(dir->d_name);
            auto pos = filename.find("logfile");
            if (pos != std::string::npos)
            {
                //printf("filename.substr(pos = %d, 3) %s\n", pos, filename.substr(pos + 7, 3).c_str());
                nextLogFileNumber = std::max(nextLogFileNumber, std::stoi(filename.substr(pos + 7, 3)));
            }
        }

        nextLogFileNumber++;
        closedir(d);
    }

    char buf[4];
    sprintf(buf, "%03d", nextLogFileNumber);
    return buf;
}

void Logger::logMsg(ELogLevel level, const char* source, const char* msg, const char* msg2 /* = nullptr */, bool bHeader /* = c_bData */)
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
	        fprintf(m_fd, "%.6f,%d,%s,%c,%s\n", timestamp, bHeader ? 1 : 0, source, c_vecLevels[level], msg);
        }
        else
        {
            fprintf(m_fd, "%.6f,%d,%s,%c,%s,%s\n", timestamp, bHeader ? 1 : 0, source, c_vecLevels[level], msg, msg2);
        }
        
        if (m_console_echo)
        {
            if (msg2 == nullptr)
            {
                printf("%.6f,%d,%s,%c,%s\n", timestamp, bHeader ? 1 : 0, source, c_vecLevels[level], msg);
            }
            else
            {
                printf("%.6f,%d,%s,%c,%s,%s\n", timestamp, bHeader ? 1 : 0, source, c_vecLevels[level], msg, msg2);
            }
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
        out += sprintf(out, "%.3f%c", data[i], i < c_numDataPts - 1 ? ',' : ' ');
 		if (out - m_formattedDoubleData.c_str() >= c_totalSz)
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
        out += sprintf(out, "%6d%c", data[i], i < c_numDataPts - 1 ? ',' : ' ');
        if (out - m_formattedIntData.c_str() >= c_totalSz)
        {
            break;
        }
    }
}
