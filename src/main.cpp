// SPDX-License-Identifier: GPL-3.0-or-later
//
// Управление напольным отоплением Uponor Smatrix (контроллер X-245) через Apple
// HomeKit на ESP32. Точка входа: сборка моста и термостатов, Wi-Fi, синхронизация
// времени, служебная диагностика.
//
// Часть проекта, работающая с шиной, портирована из компонента uponor_smatrix
// проекта ESPHome (GPLv3), поэтому проект целиком распространяется под GPLv3.
// Слой HomeKit построен на библиотеке HomeSpan (лицензия MIT). См. README и NOTICE.
#include <HomeSpan.h>
#include <WiFi.h>

// Включает автооткат ESP-IDF: прошивка стартует в состоянии PENDING_VERIFY и,
// если её не подтвердить markSketchOK(), при следующей перезагрузке загрузчик
// вернёт предыдущую версию. Ниже в loop() подтверждение выдаётся только после
// того, как Wi-Fi действительно поднялся, — иначе плата, потерявшая сеть после
// неудачной загрузки прошивки, откатится сама. Это важно: плата обычно стоит у
// контроллера X-245, и снимать её для восстановления по USB неудобно.
#include "SpanRollback.h"

#include "config.h"
#include "secrets.h"
#include "uponor_protocol.h"
#include "uponor_zone.h"

static UponorBus bus(Serial2, RS485_RX_PIN, RS485_TX_PIN);

static UponorZoneService *zone_services[ZONE_COUNT];

// Синхронизация времени: X-245 своих часов не имеет, время держит первый
// зарегистрированный на контроллере термостат. Раньше время приходило из Home
// Assistant, теперь из NTP — его настраивает веб-журнал HomeSpan (вызов
// configTzTime внутри enableWebLog).
static const uint32_t TIME_SYNC_INTERVAL_MS = 3600UL * 1000UL;
static uint32_t last_time_sync = 0;

// Состояние зон и счётчики шины — в Serial и в веб-лог на http://<ip>/status.
// Веб-лог нужен потому, что плата стоит у контроллера и USB к ней не подведён.
static const uint32_t STATE_LOG_INTERVAL_MS = 15000;
static uint32_t last_state_log = 0;

// Ресёрч-дамп всех регистров по всем адресам шины. Реже, чем state.
// Флаг DUMP_ENABLED — в config.h.
static const uint32_t DUMP_INTERVAL_MS = 30000;
static uint32_t last_dump = 0;

// Health-gate для автоотката: если за это время Wi-Fi не поднялся, перезагрузка
// вернёт предыдущую прошивку. Окно нарочно большое — точка доступа нестабильна,
// в прошлом ESP32 подключался только с третьей попытки.
static const uint32_t WIFI_HEALTH_TIMEOUT_MS = 240000;
static bool sketch_confirmed = false;

// Одноразовое стирание пары HomeKit — см. ERASE_PAIRING_ON_BOOT в config.h.
// Ждём, пока поднимутся Wi-Fi и mDNS, иначе флаг sf в Bonjour не обновится.
static const uint32_t ERASE_PAIRING_DELAY_MS = 15000;
static bool pairing_erased = false;

void setup() {
  Serial.begin(115200);

  bus.begin();
  bus.setDumpEnabled(DUMP_ENABLED);

  homeSpan.setWifiCredentials(WIFI_SSID, WIFI_PASSWORD);
  homeSpan.setPairingCode(HOMEKIT_PAIRING_CODE);
  // Пустой суффикс вместо 6-байтового ID аксессуара — имя хоста uponor.local,
  // как было в ESPHome-конфиге.
  homeSpan.setHostNameSuffix("");
  homeSpan.enableOTA();
  homeSpan.enableWebLog(50, "pool.ntp.org", TIMEZONE, "status");

  homeSpan.begin(Category::Bridges, DEVICE_NAME, DEVICE_HOSTNAME);

  SPAN_ACCESSORY();  // мост

  // По одному аксессуару на комнату (AID 2..6). Влажность НЕ выносится в
  // отдельные аксессуары: «Дом» всё равно не рисует значение датчика на плитке,
  // а лишние 5 плиток только засоряют экран. Датчик влажности вложен вторым
  // сервисом внутрь термостата (см. конструктор UponorZoneService) — плитки не
  // создаёт, но питает климатическую сводку «Дома».
  //
  // ПОРЯДОК СОЗДАНИЯ АКСЕССУАРОВ МЕНЯТЬ НЕЛЬЗЯ: HomeSpan нумерует их (AID) по
  // порядку, а «Дом» держит соответствие в кеше. Сдвиг номеров ломает привязку
  // и показывает все аксессуары как «Недоступно». Новое добавлять только в конец.
  for (size_t i = 0; i < ZONE_COUNT; i++) {
    SPAN_ACCESSORY(ZONES[i].name);
    zone_services[i] = new UponorZoneService(&bus, ZONES[i].address);
  }
}

void loop() {
  homeSpan.poll();
  bus.loop();

  // Подтвердить прошивку, как только сеть поднялась; если не поднялась за
  // отведённое окно — перезагрузиться и тем самым откатиться.
  if (!sketch_confirmed) {
    if (WiFi.status() == WL_CONNECTED) {
      // Выключить энергосбережение Wi-Fi. arduino-esp32 на классическом ESP32
      // по умолчанию ставит WIFI_PS_MIN_MODEM: плата спит между маячками точки
      // доступа, входящие пакеты копятся на роутере и приходят пачками. Это
      // давало задержку 27..670 мс с огромным джиттером и роняло OTA по
      // таймауту. Ставится после подключения — esp_wifi_set_ps() требует уже
      // поднятого Wi-Fi. Расход тока вырастает примерно на 30 мА, что для
      // платы с постоянным питанием несущественно.
      WiFi.setSleep(false);

      homeSpan.markSketchOK();
      sketch_confirmed = true;
      Serial.println("[uponor] sketch confirmed, wifi sleep off, rollback disarmed");
    } else if (millis() > WIFI_HEALTH_TIMEOUT_MS) {
      Serial.println("[uponor] no WiFi within health window -- rebooting to roll back");
      delay(200);
      ESP.restart();
    }
  }

  // Одноразовое стирание пары HomeKit: только когда сеть и mDNS уже поднялись,
  // иначе флаг sf в Bonjour останется прежним до перезагрузки.
  if (ERASE_PAIRING_ON_BOOT && !pairing_erased && sketch_confirmed && millis() > ERASE_PAIRING_DELAY_MS) {
    Serial.println("[uponor] ERASE_PAIRING_ON_BOOT: стираю сохранённые сопряжения");
    homeSpan.processSerialCommand("U");
    pairing_erased = true;
  }

  if (bus.hasTimeMaster() && (last_time_sync == 0 || millis() - last_time_sync > TIME_SYNC_INTERVAL_MS)) {
    bus.requestSendTime();
    last_time_sync = millis();
  }

  if (millis() - last_state_log > STATE_LOG_INTERVAL_MS) {
    char buf[192];

    Serial.printf("[uponor] %s\n", bus.formatStats(buf, sizeof(buf)));
    WEBLOG("%s", buf);
    Serial.printf("[uponor] %s\n", bus.formatLastBytes(buf, sizeof(buf)));
    WEBLOG("%s", buf);

    for (size_t i = 0; i < ZONE_COUNT; i++) {
      zone_services[i]->formatState(buf, sizeof(buf), ZONES[i].name);
      Serial.printf("[uponor] %s\n", buf);
      WEBLOG("%s", buf);
    }

    last_state_log = millis();
  }

  // Ресёрч-дамп: все регистры всех адресов шины, сырой hex.
  if (DUMP_ENABLED && millis() - last_dump > DUMP_INTERVAL_MS) {
    char buf[256];
    auto addrs = bus.dumpAddresses();
    for (uint32_t a : addrs) {
      bus.formatDumpLine(a, buf, sizeof(buf));
      Serial.printf("[dump] %s\n", buf);
      WEBLOG("dump %s", buf);
    }
    last_dump = millis();
  }
}
