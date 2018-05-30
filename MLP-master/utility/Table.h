#pragma once
#include "MegunoLinkProtocol.h"

class Table : protected MegunoLinkProtocol
{
public:
  Table(Print &rDestination = Serial);

  template<class TValue> void SendData(const char *RowName, TValue Value, const char *Description = NULL)
  {
    SendHeader_Set();
    m_rDestination.print(RowName);
    m_rDestination.print('|');
    m_rDestination.print(Value);
    if (Description != NULL)
    {
      m_rDestination.print('|');
      m_rDestination.print(Description);
    }
    SendDataTail();
  }

  template<class TValue> void SendData(const __FlashStringHelper *RowName, TValue Value, const char *Description = NULL)
  {
    SendHeader_Set();
    m_rDestination.print(RowName);
    m_rDestination.print('|');
    m_rDestination.print(Value);
    if (Description != NULL)
    {
      m_rDestination.print('|');
      m_rDestination.print(Description);
    }
    SendDataTail();
  }

  template<class TValue> void SendData(const __FlashStringHelper *RowName, TValue Value, const __FlashStringHelper *Description)
  {
    SendHeader_Set();
    m_rDestination.print(RowName);
    m_rDestination.print('|');
    m_rDestination.print(Value);
    if (Description != NULL)
    {
      m_rDestination.print('|');
      m_rDestination.print(Description);
    }
    SendDataTail();
  }

  void SetDescription(const char *RowName, const char *Description);
  void SetDescription(const __FlashStringHelper *RowName, const char *Description);
  void SetDescription(const __FlashStringHelper *RowName, const __FlashStringHelper *Description);

  void ClearAllRows();
  void ClearRow(const char *RowName);
  void ClearRow(const __FlashStringHelper *RowName);

  void GetData(const char *RowName);
  void GetData(const __FlashStringHelper *RowName);

private:
  void SendHeader_Set();

};

