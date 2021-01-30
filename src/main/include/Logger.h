/*
    Logging Class
    2/15/20

    Nicholas Seidl
    Modified by Scott Coursin
    
*/

#ifndef SRC_Logger_H_
#define SRC_Logger_H_

#include <stdio.h>
#include <frc\timer.h>
#include <frc/shuffleboard/Shuffleboard.h>

#include <vector>
#include <string>

const std::string c_logFileBasePath = "/tmp/logfile";
const std::string c_logFileExtension = ".csv";

// Use c_bHeader and c_bData in the bHeader argument of Logger::logMsg
constexpr bool c_bHeader = true;
constexpr bool c_bData = false;

enum ELogLevel
{
      eDebug
    , eInfo
    , eWarn
    , eError
};

using namespace std;
using namespace frc;

class EmptyType
{
};

template <typename E>
class LogDataT
{
public:
  LogDataT(const std::vector<std::string>& headerNames, bool bAddToDashboard, const std::string& dashboardPrefix)
    : m_headerNames(headerNames)
    , m_dashboardPrefix(dashboardPrefix)
    , m_bAddToDashboard(bAddToDashboard)
  {
    ShuffleboardTab& tab = Shuffleboard::GetTab("LogShadow");

    int h = 0;
    if (E::eFirstInt < E::eLastInt)
    {
      for (int i = (int)E::eFirstInt; i < (int)E::eLastInt; i++)
      {
        m_dataInt.push_back(0);
        if (m_bAddToDashboard)
        {
          std::string header = dashboardPrefix + m_headerNames[h++];
          m_netTableEntries.push_back(tab.Add(header, 0).GetEntry());
          m_netTableEntries.back().SetDouble(0.0);
        }
      }
    }

    for (int i = (int)E::eFirstDouble; i < (int)E::eLastDouble; i++)
    {
      m_dataDouble.push_back(0.0);
      if (m_bAddToDashboard)
      {
        std::string header = dashboardPrefix + m_headerNames[h++];
        m_netTableEntries.push_back(tab.Add(header, 0).GetEntry());
        m_netTableEntries.back().SetDouble(0.0);
      }
    }
  }

  int& operator[](int index)
  {
    return m_dataInt[index];
  }

  double& operator[](E index)
  {
    return m_dataDouble[(int)index - (int)E::eFirstDouble];
  }

  const std::vector<std::string>& GetHeaderNames() const
  {
    return m_headerNames;
  }

  const std::vector<int>& GetInts() const
  {
    return m_dataInt;
  }

  const std::vector<double>& GetDoubles() const
  {
    return m_dataDouble;
  }

  void UpdateDashboard()
  {
    if (m_bAddToDashboard)
    {
      int h = 0;
      if (E::eFirstInt < E::eLastInt)
      {
        for (int i = (int)E::eFirstInt; i < (int)E::eLastInt; i++)
        {
          m_netTableEntries[h++].SetDouble((double)m_dataInt[i]);
        }
      }

      for (int i = (int)E::eFirstDouble; i < (int)E::eLastDouble; i++)
      {
        m_netTableEntries[h++].SetDouble(m_dataDouble[i - (int)E::eFirstDouble]);
      }
    }
  }

  bool LoggedHeader() const { return m_bLoggedHeader; }
  void SetHeaderLogged() { m_bLoggedHeader = true; }
  void ResetHeaderLogged() { m_bLoggedHeader = false; }

private:
  std::vector<std::string> m_headerNames;
  std::string m_dashboardPrefix;
  std::vector<int> m_dataInt;
  std::vector<double> m_dataDouble;
  bool m_bAddToDashboard;
  bool m_bLoggedHeader = false;
  std::vector<nt::NetworkTableEntry> m_netTableEntries;
};

class Logger
{
    FILE* m_fd = nullptr;
    Timer m_timer;
    bool m_console_echo = false;

  public:
    Logger(bool console_echo);
    ~Logger();

    void openLog();
    void closeLog();
    std::string FindNextLogFileNumber();

    void logMsg(ELogLevel level, const char* source, const char* msg, const char* msg2 = nullptr, bool bHeader = c_bData);

    template <typename E>
    void logData(const char* source, LogDataT<E>& data)
    {
      m_formattedIntData.clear();
      m_formattedDoubleData.clear();

      auto& ints = data.GetInts();
      if (!ints.empty())
      {
        formatData(ints);
      }

      auto& doubles = data.GetDoubles();
      if (!doubles.empty())
      {
        formatData(doubles);
      }

      if (!data.LoggedHeader())
      {
          data.SetHeaderLogged();
          logHeader(source, data);
      }

      if (!ints.empty() && !doubles.empty())
      {
        logMsg(eInfo, source, m_formattedIntData.c_str(), m_formattedDoubleData.c_str());
      }
      else if (!ints.empty())
      {
        logMsg(eInfo, source, m_formattedIntData.c_str());
      }
      else
      {
        logMsg(eInfo, source, m_formattedDoubleData.c_str());
      }
      
      // Only update the dashboard once a second
      if (m_timer.HasPeriodPassed(1.0))
      {
        data.UpdateDashboard();
      }
    }

  protected:
    template <typename E>
    void logHeader(const char* source, const LogDataT<E>& data)
    {
      std::string out;
      auto& header = data.GetHeaderNames();
      out.reserve(header.size() * 20);
      for (auto& hn : header)
      {
        out += hn + ',';
      }
      out.erase(out.size() - 1, 1);           // Remove the trailing comma
      logMsg(eInfo, source, out.c_str(), nullptr, c_bHeader);
    }

    void formatData(const vector<double>& data);
    void formatData(const vector<int>& data);

    string m_formattedIntData;
    string m_formattedDoubleData;
};

#endif /* SRC_Logger_H_ */
