#!/bin/sh
# Считывает журнал с платы после аппаратного сброса.
#   ./capture.sh [секунды]
#   PORT=/dev/cu.usbserial-XXXX ./capture.sh 30   — если порт не определился сам
#
# Без сброса порт остаётся пустым: плата выводит сообщения только по таймеру, а
# приветственный баннер HomeSpan проходит один раз при запуске. Поэтому мы
# сбрасываем плату и слушаем порт с самого начала.
SECS="${1:-30}"

# Порт по умолчанию: первый найденный usbserial-порт. Можно переопределить
# переменной окружения PORT.
if [ -z "$PORT" ]; then
  PORT=$(ls /dev/cu.usbserial* /dev/cu.wchusbserial* /dev/cu.SLAB_USBtoUART* 2>/dev/null | head -1)
fi
if [ -z "$PORT" ]; then
  echo "Последовательный порт не найден. Укажите его вручную: PORT=/dev/cu.usbserial-XXXX ./capture.sh" >&2
  exit 1
fi

exec "$(dirname "$0")/.venv/bin/python" - "$PORT" "$SECS" <<'PY'
import serial, sys, time
port, secs = sys.argv[1], float(sys.argv[2])
p = serial.Serial()
p.port, p.baudrate, p.timeout = port, 115200, 0.2
p.dtr = p.rts = False
p.open()
p.setDTR(False); p.setRTS(True); time.sleep(0.2); p.setRTS(False)   # аппаратный сброс через вывод EN
p.reset_input_buffer()
t0 = time.time()
while time.time() - t0 < secs:
    d = p.read(4096)
    if d:
        sys.stdout.write(d.decode('utf-8', 'replace'))
        sys.stdout.flush()
p.close()
PY
