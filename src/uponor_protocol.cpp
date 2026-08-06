// SPDX-License-Identifier: GPL-3.0-or-later
// Логика протокола портирована из компонента uponor_smatrix проекта ESPHome
// (автор Stefan Rado, kroimon), GPLv3. См. README и NOTICE.
#include "uponor_protocol.h"

#include <ctime>

void UponorBus::begin() {
  // 512 байт, как параметр rx_buffer_size в конфигурации ESPHome: при скорости
  // 19200 бод стандартные 128 байт аппаратного буфера FIFO считываются за ~53 мс,
  // и колебания задержки основного цикла (HomeSpan обслуживает Wi-Fi и HAP)
  // приводили бы к потере пакетов.
  this->serial_.setRxBufferSize(512);
  this->serial_.begin(19200, SERIAL_8N1, this->rx_pin_, this->tx_pin_);
  this->rx_buffer_.reserve(64);
}

void UponorBus::loop() {
  const uint32_t now = millis();

  if (!this->rx_buffer_.empty() && (now - this->last_rx_ > 50)) {
    this->rx_discarded_ += this->rx_buffer_.size();
    this->rx_buffer_.clear();
  }

  while (this->serial_.available()) {
    // Контроллер опрашивает термостаты непрерывно, ~200 мс между устройствами.
    // Запоминаем момент последнего байта, чтобы вклиниться в тишину.
    this->last_rx_ = now;

    uint8_t byte = this->serial_.read();
    this->rx_bytes_++;
    this->last_bytes_[this->last_bytes_len_ % sizeof(this->last_bytes_)] = byte;
    this->last_bytes_len_++;

    if (this->parseByte_(byte)) {
      this->rx_packets_++;
      this->rx_buffer_.clear();
    }
  }

  // Окно отправки: шина молчит 50..100 мс и с нашей прошлой посылки прошло >200 мс.
  if (this->rx_buffer_.empty() && (now - this->last_rx_ > 50) && (now - this->last_rx_ < 100) &&
      (now - this->last_tx_ > 200)) {
    if (this->send_time_requested_ && this->tx_queue_.empty() && this->doSendTime_()) {
      this->send_time_requested_ = false;
    }

    if (!this->tx_queue_.empty()) {
      auto packet = std::move(this->tx_queue_.front());
      this->tx_queue_.pop();

      this->serial_.write(packet.data(), packet.size());
      this->serial_.flush();

      this->last_tx_ = now;
    }
  }
}

const char *UponorBus::formatStats(char *buf, size_t len) const {
  snprintf(buf, len, "bus rx=%u bytes, packets=%u, discarded=%u, buffered=%u, timeMaster=%s", this->rx_bytes_,
           this->rx_packets_, this->rx_discarded_, (unsigned) this->rx_buffer_.size(),
           this->time_device_address_ ? "yes" : "no");
  return buf;
}

const char *UponorBus::formatLastBytes(char *buf, size_t len) const {
  if (this->last_bytes_len_ == 0) {
    snprintf(buf, len, "last bytes: (ничего не принято)");
    return buf;
  }

  size_t n = this->last_bytes_len_ < sizeof(this->last_bytes_) ? this->last_bytes_len_ : sizeof(this->last_bytes_);
  size_t start = (this->last_bytes_len_ >= sizeof(this->last_bytes_))
                     ? (this->last_bytes_len_ % sizeof(this->last_bytes_))
                     : 0;

  size_t off = snprintf(buf, len, "last bytes:");
  for (size_t i = 0; i < n && off < len; i++) {
    off += snprintf(buf + off, len - off, " %02X", this->last_bytes_[(start + i) % sizeof(this->last_bytes_)]);
  }
  return buf;
}

std::vector<uint32_t> UponorBus::dumpAddresses() const {
  std::vector<uint32_t> out;
  for (const auto &kv : this->dump_seen_)
    out.push_back(kv.first);
  return out;
}

const char *UponorBus::formatDumpLine(uint32_t addr, char *buf, size_t len) const {
  size_t off = snprintf(buf, len, "0x%08X:", addr);
  auto it = this->dump_seen_.find(addr);
  if (it != this->dump_seen_.end()) {
    for (const auto &reg : it->second) {
      if (off < len)
        off += snprintf(buf + off, len - off, " %02X=%04X", reg.first, reg.second);
    }
  }
  return buf;
}

bool UponorBus::parseByte_(uint8_t byte) {
  this->rx_buffer_.push_back(byte);
  const uint8_t *packet = this->rx_buffer_.data();
  size_t packet_len = this->rx_buffer_.size();

  if (packet_len < 7) {
    return false;
  }

  uint32_t device_address = uponor_encode_uint32(packet[0], packet[1], packet[2], packet[3]);
  uint16_t crc = uponor_encode_uint16(packet[packet_len - 1], packet[packet_len - 2]);

  if (crc != uponor_crc16(packet, packet_len - 2)) {
    // CRC не сошёлся — возможно, пакет ещё не дочитан.
    return false;
  }

  size_t data_len = (packet_len - 6) / 3;
  if (data_len == 0) {
    return true;
  }

  UponorSmatrixData data[data_len];
  for (size_t i = 0; i < data_len; i++) {
    data[i].id = packet[(i * 3) + 4];
    data[i].value = uponor_encode_uint16(packet[(i * 3) + 5], packet[(i * 3) + 6]);
  }

  // Ресёрч-дамп: копим последние значения регистров по ВСЕМ адресам, включая
  // незарегистрированные (контроллер и т.п.). Только когда дамп включён.
  if (this->dump_enabled_) {
    for (size_t i = 0; i < data_len; i++) {
      this->dump_seen_[device_address][data[i].id] = data[i].value;
    }
  }

  // Первый зарегистрированный на контроллере термостат — источник времени.
  // Опознаём его по пакету, в котором есть и температура комнаты, и дата.
  if (this->time_device_address_ == 0 && data_len >= 2) {
    bool found_temperature = false;
    bool found_time = false;
    for (size_t i = 0; i < data_len; i++) {
      if (data[i].id == UPONOR_ID_ROOM_TEMP)
        found_temperature = true;
      if (data[i].id == UPONOR_ID_DATETIME1)
        found_time = true;
      if (found_temperature && found_time) {
        Serial.printf("[uponor] time master: 0x%08X\n", device_address);
        this->time_device_address_ = device_address;
        break;
      }
    }
  }

  for (auto *zone : this->zones_) {
    if (zone->address() == device_address) {
      zone->onDeviceData(data, data_len);
    }
  }

  return true;
}

bool UponorBus::send(uint32_t device_address, const UponorSmatrixData *data, size_t data_len) {
  if (device_address == 0 || data == nullptr || data_len == 0)
    return false;

  // Все поля big-endian, кроме little-endian контрольной суммы.
  std::vector<uint8_t> packet;
  packet.reserve(6 + 3 * data_len);

  packet.push_back(device_address >> 24);
  packet.push_back(device_address >> 16);
  packet.push_back(device_address >> 8);
  packet.push_back(device_address >> 0);

  for (size_t i = 0; i < data_len; i++) {
    packet.push_back(data[i].id);
    packet.push_back(data[i].value >> 8);
    packet.push_back(data[i].value >> 0);
  }

  uint16_t crc = uponor_crc16(packet.data(), packet.size());
  packet.push_back(crc >> 0);
  packet.push_back(crc >> 8);

  this->tx_queue_.push(std::move(packet));
  return true;
}

bool UponorBus::doSendTime_() {
  if (this->time_device_address_ == 0)
    return false;

  time_t raw = time(nullptr);
  struct tm tm_now;
  localtime_r(&raw, &tm_now);
  if (tm_now.tm_year + 1900 < 2020)
    return false;  // NTP ещё не синхронизировался

  uint8_t year = (tm_now.tm_year + 1900) - 2000;
  uint8_t month = tm_now.tm_mon + 1;
  // tm_wday: 0=воскресенье. У Uponor 0..6 начиная с понедельника.
  uint8_t day_of_week = (tm_now.tm_wday == 0) ? 6 : (tm_now.tm_wday - 1);

  uint16_t time1 = (year & 0x7F) << 7 | (month & 0x0F) << 3 | (day_of_week & 0x07);
  uint16_t time2 = (tm_now.tm_mday & 0x1F) << 11 | (tm_now.tm_hour & 0x1F) << 6 | (tm_now.tm_min & 0x3F);
  uint16_t time3 = tm_now.tm_sec;

  Serial.printf("[uponor] sending time %04d-%02d-%02d %02d:%02d:%02d\n", tm_now.tm_year + 1900, month, tm_now.tm_mday,
                tm_now.tm_hour, tm_now.tm_min, tm_now.tm_sec);

  UponorSmatrixData data[] = {{UPONOR_ID_DATETIME1, time1}, {UPONOR_ID_DATETIME2, time2}, {UPONOR_ID_DATETIME3, time3}};
  return this->send(this->time_device_address_, data, sizeof(data) / sizeof(data[0]));
}
