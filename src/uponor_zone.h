// SPDX-License-Identifier: GPL-3.0-or-later
//
// Одна зона (комната) системы Uponor как термостат HomeKit: чтение температуры,
// влажности, уставки и режима с шины и запись уставки обратно.
//
// Логика управления уставкой, режимом и ECO портирована из компонента
// uponor_smatrix проекта ESPHome (автор Stefan Rado, kroimon), GPLv3.
// См. файлы README и NOTICE.
#pragma once

#include <HomeSpan.h>

#include "uponor_protocol.h"

/// Одна зона (термостат) системы Uponor как HomeKit-термостат.
///
/// Влажность публикуется ДВАЖДЫ в пределах одного аксессуара: как необязательная
/// характеристика самого сервиса Thermostat (видна в панели термостата) и как
/// вложенный сервис HumiditySensor (питает климатическую сводку «Дома», плитку
/// не создаёт). Значение с шины одно, пишется в обе характеристики.
struct UponorZoneService : Service::Thermostat, UponorZone {
  UponorZoneService(UponorBus *bus, uint32_t address);

  uint32_t address() const override { return this->address_; }
  void onDeviceData(const UponorSmatrixData *data, size_t data_len) override;

  boolean update() override;
  void loop() override;

  /// Формирует состояние зоны в buf — замена блока interval из 03-verify.yaml.
  const char *formatState(char *buf, size_t len, const char *name) const;

 private:
  void publishTargetTemperature_();

  /// ECO-откат в сырых единицах шины СО ЗНАКОМ, зависящим от режима.
  ///
  /// Мануал Smatrix Base PULSE, меню «03 ECO mode setback temperature»:
  /// «The setting adjusts the current setpoint with the set value. In heating
  /// mode the setpoint is reduced, and in cooling mode it is increased.»
  ///
  /// То есть в регистре 0x3B лежит БАЗОВАЯ (comfort) уставка, а термостат сам
  /// применяет откат: в нагреве минус, в охлаждении плюс. Возвращает 0, если ECO
  /// не активен. Одно место на чтение и запись, чтобы знак не разъехался.
  int16_t ecoDeltaRaw_() const;

  UponorBus *bus_;
  uint32_t address_;

  SpanCharacteristic *current_temp_;
  SpanCharacteristic *target_temp_;
  SpanCharacteristic *current_state_;
  SpanCharacteristic *target_state_;
  SpanCharacteristic *humidity_;                  // внутри сервиса Thermostat
  SpanCharacteristic *humidity_sensor_{nullptr};  // в отдельном аксессуаре

  // Состояние, вычитанное с шины.
  uint32_t last_data_{0};
  uint16_t target_temp_raw_{0};
  uint16_t eco_setback_raw_{0x0048};
  bool eco_active_{false};
  bool cooling_mode_{false};
};
