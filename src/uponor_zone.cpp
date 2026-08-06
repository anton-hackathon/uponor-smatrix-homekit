// SPDX-License-Identifier: GPL-3.0-or-later
// Логика управления уставкой, режимом и ECO портирована из компонента
// uponor_smatrix проекта ESPHome (автор Stefan Rado, kroimon), GPLv3.
// См. README и NOTICE.
#include "uponor_zone.h"

#include "config.h"  // UPONOR_TARGET_TEMP_OFFSET

// HomeKit: CurrentHeatingCoolingState 0=idle, 1=heat, 2=cool.
// TargetHeatingCoolingState 0=off, 1=heat, 2=cool, 3=auto.
static const uint8_t HK_STATE_IDLE = 0;
static const uint8_t HK_STATE_HEAT = 1;
static const uint8_t HK_STATE_COOL = 2;

static const float TARGET_TEMP_STEP = 0.5f;

UponorZoneService::UponorZoneService(UponorBus *bus, uint32_t address) : bus_(bus), address_(address) {
  this->current_state_ = new Characteristic::CurrentHeatingCoolingState(HK_STATE_IDLE);
  this->target_state_ = new Characteristic::TargetHeatingCoolingState(HK_STATE_HEAT);
  this->current_temp_ = new Characteristic::CurrentTemperature(20.0f);
  this->target_temp_ = new Characteristic::TargetTemperature(20.0f);
  new Characteristic::TemperatureDisplayUnits(0);  // 0 = Цельсий
  this->humidity_ = new Characteristic::CurrentRelativeHumidity(0.0f);

  // Диапазоны обязаны укладываться в спецификацию HAP, иначе «Дом» может не
  // отрисовать панель управления термостатом:
  //   CurrentTemperature — спека допускает 0..100
  //   TargetTemperature  — спека допускает 10..38
  // Термостат Uponor по регистрам 0x37/0x38 отдаёт 5..35, но 5 °C ниже
  // разрешённого HomeKit минимума, поэтому снизу берём 10. Практической
  // потери нет: уставка ниже 10 °C для отопления смысла не имеет, а из-за
  // поправки −2 запрос 5 °C всё равно уходил в шину как 3 °C, ниже минимума
  // самого устройства. Сверху остаётся 35 — предел устройства.
  this->current_temp_->setRange(0.0f, 60.0f);
  this->target_temp_->setRange(10.0f, 35.0f, TARGET_TEMP_STEP);

  // Набор режимов НЕ ограничиваем (оставляем стандартные OFF/HEAT/COOL/AUTO):
  // урезание через setValidValues уже ломало открытие панели, эталонный
  // термостат HomeSpan его тоже не трогает.
  //
  // Режим не управляется — его задаёт контроллер X-245 на всю систему, в шину
  // он не пишется. Поэтому делаем характеристику ТОЛЬКО ДЛЯ ЧТЕНИЯ: убираем
  // право записи PW. «Дом» перестаёт слать смену режима, переключатель
  // становится неактивным и лишь отражает фактический режим шины. Это заодно
  // убирает необходимость откатывать чужую запись в loop().
  //
  // Сохраняем PR+EV: чтение и уведомления об изменении обязательны, снимаем
  // только запись.
  this->target_state_->removePerms(PW);

  // Датчик влажности — ВТОРЫМ сервисом внутри этого же аксессуара, а не
  // отдельным аксессуаром. Так «Дом» не создаёт лишнюю плитку (значение датчика
  // он на плитке всё равно не рисует), но берёт влажность в климатическую
  // сводку. Влажность пишется и сюда, и в humidity_ внутри термостата (для
  // панели термостата).
  //
  // ВАЖНО: этот сервис создаётся ПОСЛЕДНИМ. HomeSpan цепляет каждую новую
  // характеристику к последнему созданному сервису, поэтому все характеристики
  // термостата должны быть объявлены выше.
  new Service::HumiditySensor();
  this->humidity_sensor_ = new Characteristic::CurrentRelativeHumidity(0.0f);

  this->bus_->registerZone(this);
}

void UponorZoneService::onDeviceData(const UponorSmatrixData *data, size_t data_len) {
  for (size_t i = 0; i < data_len; i++) {
    switch (data[i].id) {
      case UPONOR_ID_TARGET_TEMP:
        // Недопустимое значение контроллер использует, чтобы запросить
        // уставку у термостата — его игнорируем.
        if (data[i].value != UPONOR_INVALID_VALUE)
          this->target_temp_raw_ = data[i].value;
        break;
      case UPONOR_ID_ECO_SETBACK:
        this->eco_setback_raw_ = data[i].value;
        break;
      case UPONOR_ID_DEMAND: {
        this->cooling_mode_ = data[i].value & 0x1000;
        bool active = data[i].value & 0x0040;
        uint8_t state = !active ? HK_STATE_IDLE : (this->cooling_mode_ ? HK_STATE_COOL : HK_STATE_HEAT);
        if (this->current_state_->getVal<uint8_t>() != state)
          this->current_state_->setVal(state);

        // Целевой режим только отражает фактический (характеристика read-only).
        // Обновляем здесь, а не в loop(), — со стороны платы setVal разрешён.
        uint8_t target = this->cooling_mode_ ? HK_STATE_COOL : HK_STATE_HEAT;
        if (this->target_state_->getVal<uint8_t>() != target)
          this->target_state_->setVal(target);
        break;
      }
      case UPONOR_ID_MODE1:
        this->eco_active_ = data[i].value & 0x0008;
        break;
      case UPONOR_ID_ROOM_TEMP: {
        float temp = uponor_raw_to_celsius(data[i].value);
        if (!std::isnan(temp))
          this->current_temp_->setVal(temp);
        break;
      }
      case UPONOR_ID_HUMIDITY: {
        float hum = static_cast<float>(data[i].value & 0x00FF);
        this->humidity_->setVal(hum);
        if (this->humidity_sensor_ != nullptr)
          this->humidity_sensor_->setVal(hum);
        break;
      }
    }
  }

  this->last_data_ = millis();
}

int16_t UponorZoneService::ecoDeltaRaw_() const {
  if (!this->eco_active_)
    return 0;
  int16_t setback = static_cast<int16_t>(this->eco_setback_raw_);
  return this->cooling_mode_ ? setback : -setback;
}

void UponorZoneService::publishTargetTemperature_() {
  // Эффективная уставка = базовая (0x3B) + откат со знаком по режиму.
  int32_t raw = static_cast<int32_t>(this->target_temp_raw_) + this->ecoDeltaRaw_();
  raw = constrain(raw, 0, 0x7FFE);  // не выйти за диапазон и не попасть в 0x7FFF
  float temp = uponor_raw_to_celsius(static_cast<uint16_t>(raw)) + UPONOR_TARGET_TEMP_OFFSET;
  if (std::isnan(temp))
    return;

  temp = roundf(temp / TARGET_TEMP_STEP) * TARGET_TEMP_STEP;
  // Держим в границах характеристики: setVal() вне диапазона HomeSpan считает
  // ошибкой. Границы должны совпадать с setRange() в конструкторе.
  temp = constrain(temp, 10.0f, 35.0f);
  if (this->target_temp_->getVal<float>() != temp)
    this->target_temp_->setVal(temp);
}

void UponorZoneService::loop() {
  // Публикуем уставку, когда все пакеты обновления разобраны: ECO-состояние и
  // само значение могут прийти в разных пакетах.
  if (this->last_data_ != 0 && (millis() - this->last_data_ > 100) && this->target_temp_raw_ != 0) {
    this->publishTargetTemperature_();
    this->last_data_ = 0;
  }
}

const char *UponorZoneService::formatState(char *buf, size_t len, const char *name) const {
  // base — сырая уставка с шины, ecoDelta — применённый откат со знаком.
  // Нужны, чтобы при первом включении ECO сразу увидеть, верен ли знак.
  snprintf(buf, len, "%-10s 0x%08X temp=%.1f target=%.1f hum=%.0f mode=%s state=%u%s base=%u ecoDelta=%d", name,
           this->address_, this->current_temp_->getVal<float>(), this->target_temp_->getVal<float>(),
           this->humidity_->getVal<float>(), this->cooling_mode_ ? "COOL" : "HEAT",
           this->current_state_->getVal<uint8_t>(), this->eco_active_ ? " ECO" : "", this->target_temp_raw_,
           (int) this->ecoDeltaRaw_());
  return buf;
}

boolean UponorZoneService::update() {
  if (this->target_temp_->updated()) {
    float requested = this->target_temp_->getNewVal<float>();

    // Обратная операция к publishTargetTemperature_(): в шину уходит БАЗОВАЯ
    // уставка, поэтому откат снимаем — термостат применит его сам, со знаком
    // по режиму.
    int32_t raw = static_cast<int32_t>(uponor_celsius_to_raw(requested - UPONOR_TARGET_TEMP_OFFSET)) -
                  this->ecoDeltaRaw_();
    raw = constrain(raw, 0, 0x7FFE);

    // Термостат реагирует только если перед значением отправить нулевую уставку.
    UponorSmatrixData data[] = {{UPONOR_ID_TARGET_TEMP, 0},
                                {UPONOR_ID_TARGET_TEMP, static_cast<uint16_t>(raw)}};
    this->bus_->send(this->address_, data, sizeof(data) / sizeof(data[0]));

    Serial.printf("[uponor] 0x%08X setpoint -> %.1f C (raw %d, ecoDelta %d)\n", this->address_, requested,
                  (int) raw, (int) this->ecoDeltaRaw_());
  }

  return true;
}
