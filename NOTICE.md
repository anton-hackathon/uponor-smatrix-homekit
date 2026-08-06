# Атрибуция и лицензии

Этот проект распространяется под лицензией **GNU General Public License v3.0**
(см. файл [LICENSE](LICENSE)). Ниже перечислены сторонние работы, на которые он
опирается, и их лицензии.

## Протокол шины Uponor Smatrix

Разбор протокола и карта регистров основаны на компоненте `uponor_smatrix`
проекта **ESPHome** (автор Stefan Rado, псевдоним kroimon). Логика работы с шиной
(контрольная сумма, разбор пакетов, управление уставкой, режимом и ECO)
портирована на HomeSpan из этого компонента.

Весь код на C++ в проекте ESPHome распространяется под лицензией **GPLv3**.
Поскольку наш код является производной работой, он также распространяется под
GPLv3 — это требование лицензии, а не свободный выбор.

- ESPHome: https://esphome.io/
- Компонент: https://esphome.io/components/uponor_smatrix/
- Исходный код: https://github.com/esphome/esphome (каталог
  `esphome/components/uponor_smatrix/`)

## Библиотека HomeKit — HomeSpan

Слой Apple HomeKit построен на библиотеке **HomeSpan** (автор Gregg E. Berman),
которая распространяется под лицензией **MIT**. MIT совместима с GPLv3, поэтому
включение HomeSpan в проект под GPLv3 допустимо.

- HomeSpan: https://github.com/HomeSpan/HomeSpan

## Документация Uponor

Расшифровка поведения системы (поправка при переключении нагрев/охлаждение,
режим ECO, настройки термостата, запрет охлаждения по комнатам) сверена с
официальной документацией Uponor. Сам протокол шины в документации не описан —
он результат независимого исследования (реверс-инжиниринга).

- Руководство «Uponor Smatrix Base PULSE — Installation and operation manual»
- Каталожная страница контроллера Uponor Smatrix Base Pulse X-245

«Uponor» и «Smatrix» — товарные знаки Uponor Corporation. Проект не связан с
Uponor и не одобрен этой компанией.

## Apple HomeKit

«Apple», «HomeKit», «HomePod», «Apple TV» и приложение «Дом» — товарные знаки
Apple Inc. Проект не связан с Apple и не одобрен этой компанией.
