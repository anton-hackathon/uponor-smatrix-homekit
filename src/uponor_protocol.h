// SPDX-License-Identifier: GPL-3.0-or-later
//
// Протокол шины Uponor Smatrix: разбор и сборка пакетов, контрольная сумма,
// очередь передачи, карта регистров, определение ведущего термостата времени.
//
// Логика протокола портирована из компонента uponor_smatrix проекта ESPHome
// (автор Stefan Rado, kroimon), распространяемого под лицензией GPLv3.
// Поэтому и этот проект распространяется под GPLv3. См. файлы README и NOTICE.
#pragma once

#include <Arduino.h>

#include <cmath>
#include <cstdint>
#include <map>
#include <queue>
#include <vector>

// Карта регистров шины Uponor Smatrix Base Pulse X-245.
// Значения температур лежат в десятых долях Фаренгейта.
static const uint8_t UPONOR_ID_DATETIME1 = 0x08;
static const uint8_t UPONOR_ID_DATETIME2 = 0x09;
static const uint8_t UPONOR_ID_DATETIME3 = 0x0A;
static const uint8_t UPONOR_ID_OUTDOOR_TEMP = 0x2D;
static const uint8_t UPONOR_ID_TARGET_TEMP_MIN = 0x37;
static const uint8_t UPONOR_ID_TARGET_TEMP_MAX = 0x38;
static const uint8_t UPONOR_ID_TARGET_TEMP = 0x3B;
static const uint8_t UPONOR_ID_ECO_SETBACK = 0x3C;
static const uint8_t UPONOR_ID_DEMAND = 0x3D;
static const uint8_t UPONOR_ID_MODE1 = 0x3E;
static const uint8_t UPONOR_ID_MODE2 = 0x3F;
static const uint8_t UPONOR_ID_ROOM_TEMP = 0x40;
static const uint8_t UPONOR_ID_EXTERNAL_TEMP = 0x41;
static const uint8_t UPONOR_ID_HUMIDITY = 0x42;
static const uint8_t UPONOR_ID_REQUEST = 0xFF;

static const uint16_t UPONOR_INVALID_VALUE = 0x7FFF;

// Поправка уставки UPONOR_TARGET_TEMP_OFFSET вынесена в config.h — это
// калибровка под конкретную систему, а не свойство протокола. Здесь, в
// протокольном слое, она не используется (raw_to_celsius её не трогает);
// применяется только в HomeKit-слое (uponor_zone.cpp).

struct UponorSmatrixData {
  uint8_t id;
  uint16_t value;
};

inline uint16_t uponor_encode_uint16(uint8_t msb, uint8_t lsb) {
  return (static_cast<uint16_t>(msb) << 8) | lsb;
}

inline uint32_t uponor_encode_uint32(uint8_t b1, uint8_t b2, uint8_t b3, uint8_t b4) {
  return (static_cast<uint32_t>(b1) << 24) | (static_cast<uint32_t>(b2) << 16) | (static_cast<uint32_t>(b3) << 8) | b4;
}

/// CRC-16/MODBUS: init 0xFFFF, обратный полином 0xA001, без финального XOR.
inline uint16_t uponor_crc16(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  while (len--) {
    crc ^= *data++;
    for (uint8_t i = 0; i < 8; i++) {
      crc = (crc & 0x0001) ? ((crc >> 1) ^ 0xA001) : (crc >> 1);
    }
  }
  return crc;
}

inline float uponor_raw_to_celsius(uint16_t raw) {
  return (raw == UPONOR_INVALID_VALUE) ? NAN : ((raw / 10.0f) - 32.0f) / 1.8f;
}

inline uint16_t uponor_celsius_to_raw(float celsius) {
  return std::isnan(celsius) ? UPONOR_INVALID_VALUE
                             : static_cast<uint16_t>(lroundf((celsius * 1.8f + 32.0f) * 10.0f));
}

class UponorZone;

class UponorBus {
 public:
  UponorBus(HardwareSerial &serial, int rx_pin, int tx_pin) : serial_(serial), rx_pin_(rx_pin), tx_pin_(tx_pin) {}

  void begin();
  void loop();

  void registerZone(UponorZone *zone) { this->zones_.push_back(zone); }

  bool send(uint32_t device_address, const UponorSmatrixData *data, size_t data_len);

  /// Просит отправить время в шину при следующей паузе. X-245 своих часов не
  /// имеет — время держит первый зарегистрированный на контроллере термостат,
  /// его адрес определяется автоматически по пакету, содержащему одновременно
  /// температуру комнаты и дату.
  void requestSendTime() { this->send_time_requested_ = true; }
  bool hasTimeMaster() const { return this->time_device_address_ != 0; }

  /// Диагностика приёма: отличает «байтов нет» (железо/пины) от
  /// «байты есть, но пакеты не собираются» (CRC/обрывы). Пишет строку в buf и
  /// возвращает его, чтобы вызывающий отправил её и в Serial, и в веб-лог.
  const char *formatStats(char *buf, size_t len) const;

  /// Последние принятые байты в hex — для случая «байты идут, но мусор».
  const char *formatLastBytes(char *buf, size_t len) const;

  /// РЕСЁРЧ-ДАМП. Агрегирует по КАЖДОМУ адресу на шине (не только по
  /// зарегистрированным зонам — наружная температура и время могут лететь на
  /// адрес контроллера) последнее значение каждого виденного регистра.
  /// dumpAddresses() — все адреса; formatDumpLine() — регистры одного адреса в
  /// сыром hex.
  ///
  /// По умолчанию ВЫКЛЮЧЕН: пока dump_enabled_ == false, регистры не копятся
  /// (нет накладных расходов) и вывод не идёт. Включается setDumpEnabled(true) —
  /// см. флаг DUMP_ENABLED в main.cpp. Ресёрч 2026-08-05 показал: ничего нового
  /// полезного на шине нет (наружная 0x2D=7FFF, датчиков пола нет, ECO=0), так
  /// что в норме держим выключенным.
  void setDumpEnabled(bool enabled) { this->dump_enabled_ = enabled; }
  std::vector<uint32_t> dumpAddresses() const;
  const char *formatDumpLine(uint32_t addr, char *buf, size_t len) const;

 private:
  bool parseByte_(uint8_t byte);
  bool doSendTime_();

  HardwareSerial &serial_;
  int rx_pin_;
  int tx_pin_;

  std::vector<UponorZone *> zones_;
  std::vector<uint8_t> rx_buffer_;
  std::queue<std::vector<uint8_t>> tx_queue_;

  uint32_t last_rx_{0};
  uint32_t last_tx_{0};

  uint32_t time_device_address_{0};
  bool send_time_requested_{false};

  uint32_t rx_bytes_{0};
  uint32_t rx_packets_{0};
  uint32_t rx_discarded_{0};
  uint8_t last_bytes_[16]{};
  size_t last_bytes_len_{0};

  // Ресёрч-дамп: адрес → (регистр → последнее значение). Копится только при
  // dump_enabled_.
  bool dump_enabled_{false};
  std::map<uint32_t, std::map<uint8_t, uint16_t>> dump_seen_;
};

/// Зона на шине: получает разобранные данные пакетов своего адреса.
class UponorZone {
 public:
  virtual ~UponorZone() = default;
  virtual uint32_t address() const = 0;
  virtual void onDeviceData(const UponorSmatrixData *data, size_t data_len) = 0;
};
