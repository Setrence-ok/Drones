"""
Основная программа управления дроном с уведомлениями
"""
import sys
import time
import threading
from pymavlink import mavutil

# Импорт модулей
from drone_monitor import DroneState, monitor_loop
from calculations import Point, format_distance, haversine_distance
from mission_tracker import MissionTracker
from flight_control import set_mode_guided, arm, takeoff, land, goto_position

# --- Глобальные переменные для управления потоками ---
stop_monitoring = False
monitor_thread = None
# ----------------------------------------------------

def print_drone_status(state: DroneState, title: str = "ТЕКУЩЕЕ СОСТОЯНИЕ"):
    """Вывод форматированного статуса дрона"""
    print("\n" + "=" * 60)
    print(f"📊 {title}")
    print("=" * 60)
    print(f"📍 Координаты: {state.lat_deg:.6f}, {state.lon_deg:.6f}")
    print(f"📏 Высота (относительная): {state.alt_rel_m:.1f} м")
    print(f"⚡ Батарея: {state.battery_voltage_v:.1f} В ({state.battery_remaining_pct:.0f}%)")
    print(f"🛩️  Режим: {state.mode}")
    print(f"🔧 ARM: {'ВКЛ' if state.armed else 'ВЫКЛ'}")
    print(f"🏎️  Скорость: {state.ground_speed:.1f} м/с")
    print(f"🧭 Курс: {state.heading:.0f}°")
    print(f"⏱️  Время полета: {state.flight_time:.0f} с")
    print("=" * 60)

def connect_to_simulator():
    """Подключение к симулятору"""
    print("=" * 70)
    print("🚁 ПРОГРАММА УПРАВЛЕНИЯ ДРОНОМ С ИНТЕЛЛЕКТУАЛЬНЫМИ УВЕДОМЛЕНИЯМИ")
    print("=" * 70)

    print("\n1. 📡 Подключение к симулятору...")

    # Пробуем разные порты SITL
    ports_to_try = [
        'tcp:127.0.0.1:5762',  # Основной порт SITL
        'tcp:127.0.0.1:5760',  # Стандартный порт SITL
        'tcp:127.0.0.1:5763',  # Дополнительный порт
    ]

    master = None

    for port in ports_to_try:
        print(f"   Пробуем {port}...")
        try:
            master = mavutil.mavlink_connection(
                port,
                autoreconnect=True,
                source_system=255,
                source_component=0,
                retries=3
            )

            # Ждем ответ от автопилота
            print("   Ожидаем heartbeat...")
            heartbeat = master.wait_heartbeat(timeout=5)

            if heartbeat is not None:
                print(f"✅ Успешно подключено к {port}")
                print(f"   Система: {heartbeat.get_srcSystem()}, Компонент: {heartbeat.get_srcComponent()}")

                # Устанавливаем целевую систему
                master.target_system = heartbeat.get_srcSystem()
                master.target_component = heartbeat.get_srcComponent()
                break
            else:
                print(f"   ❌ Нет ответа от {port}")
                if master:
                    master.close()
                master = None

        except Exception as e:
            print(f"   ❌ Ошибка: {e}")
            if master:
                master.close()
            master = None

    if master is None:
        print("❌ Не удалось подключиться ни к одному порту")
        return None

    return master

def wait_for_gps(state: DroneState, timeout: int = 10) -> bool:
    """Ожидание получения GPS координат"""
    print("\n2. 📍 Ожидание GPS координат и корректных высот...")

    start_time = time.time()

    for i in range(timeout):
        # Проверяем, что координаты не нулевые
        if (abs(state.lat_deg) > 0.0001 and abs(state.lon_deg) > 0.0001 and
            state.gps_fix_type >= 3):

            print(f"✅ GPS координаты получены:")
            print(f"   Координаты: {state.lat_deg:.6f}, {state.lon_deg:.6f}")
            print(f"   Высота REL: {state.alt_rel_m:.1f} м")
            print(f"   GPS фиксация: {state.gps_fix_type}D, спутники: {state.gps_satellites}")
            return True

        print(f"   Ожидание GPS... {i+1}/{timeout} сек")
        time.sleep(1)

    print("⚠️  GPS не получен, используем координаты SITL по умолчанию")
    state.lat_deg = -35.399965
    state.lon_deg = 149.198456
    return True

def setup_mission_tracker(state: DroneState, tracker: MissionTracker):
    """Настройка трекера миссии (без загрузки в автопилот)"""
    print("\n3. 🗺️  Настройка миссии в трекере...")

    # Используем текущие координаты или координаты SITL по умолчанию
    home_lat = state.lat_deg if abs(state.lat_deg) > 0.0001 else -35.399965
    home_lon = state.lon_deg if abs(state.lon_deg) > 0.0001 else 149.198456

    print(f"   Домашняя точка: {home_lat:.6f}, {home_lon:.6f}")

    # Маленький оффсет для теста
    offset = 0.00045  # ~50 метров

    # Добавляем точки миссии в трекер
    tracker.add_waypoint(home_lat, home_lon, 20, "СТАРТ (взлет)")
    tracker.add_waypoint(home_lat + offset, home_lon + offset, 30, "ТОЧКА 1")
    tracker.add_waypoint(home_lat - offset, home_lon + offset * 1.5, 40, "ТОЧКА 2")
    tracker.add_waypoint(home_lat + offset * 1.5, home_lon - offset, 50, "ТОЧКА 3")
    tracker.add_waypoint(home_lat, home_lon, 0, "ФИНИШ (посадка)")

    print(f"   Создано {len(tracker.waypoints)} точек миссии в трекере")
    print("   📝 Миссия будет выполнена через команды GUIDED (без загрузки в автопилот)")

    return True

def execute_mission_guided(state: DroneState, master, tracker: MissionTracker):
    """Выполнение миссии через команды GUIDED"""
    print("\n4. 🚀 Выполнение миссии через GUIDED...")
    print("=" * 60)
    print("🎯 ПЛАН МИССИИ:")
    print("  1. Взлет на 20м")
    print("  2. Точка 1 (30м)")
    print("  3. Точка 2 (40м)")
    print("  4. Точка 3 (50м)")
    print("  5. Посадка")
    print("=" * 60)

    # 1. Проверяем режим
    print(f"\n📋 Проверка режима: {state.mode}")
    if state.mode != "GUIDED":
        print("   Перевод в режим GUIDED...")
        if not set_mode_guided(master):
            print("   ❌ Не удалось перевести в GUIDED режим")
            return False
        time.sleep(2)
        print(f"   ✅ Режим установлен: {state.mode}")

    # 2. ARM двигателей
    print("\n🔧 ARM двигателей...")
    if not arm(master):
        print("   ❌ Не удалось выполнить ARM")
        return False

    # Ждем ARM
    print("   Ожидание ARM...")
    for i in range(10):
        if state.armed:
            print("   ✅ Дрон ARM")
            break
        print(f"   Ожидание... {i+1}/10")
        time.sleep(1)
    else:
        print("   ❌ Таймаут ожидания ARM")
        return False

    # 3. Взлет на 20 метров
    print("\n🛫 Взлет на 20 метров...")
    if not takeoff(master, 20.0):
        print("   ❌ Не удалось выполнить взлет")
        return False

    # Ждем достижения высоты
    print("   Ожидание набора высоты...")
    for i in range(30):
        if state.alt_rel_m >= 18.0:
            print(f"   ✅ Высота достигнута: {state.alt_rel_m:.1f} м")
            break
        print(f"   Текущая высота: {state.alt_rel_m:.1f} м")
        time.sleep(1)
    else:
        print("   ❌ Таймаут набора высоты")
        return False

    # Начинаем миссию
    tracker.start_mission()
    tracker.waypoints[0].reached = True  # Отмечаем взлет как выполненный

    # 4. Облет точек маршрута
    for point_index in range(1, len(tracker.waypoints) - 1):  # Пропускаем взлет и посадку
        waypoint = tracker.waypoints[point_index]

        print("\n" + "🎯" * 30)
        print(f"🎯 ПЕРЕЛЕТ К ТОЧКЕ: {waypoint.name}")
        print("🎯" * 30)

        # Выводим информацию перед началом движения
        current_pos = Point(state.lat_deg, state.lon_deg, state.alt_rel_m)
        distance = haversine_distance(current_pos, waypoint.point)
        waypoint.initial_distance = distance

        print(f"   📏 Расстояние до цели: {format_distance(distance)}")
        print(f"   🎯 Целевая высота: {waypoint.target_altitude} м")
        print(f"   📍 Координаты цели: {waypoint.point.lat:.6f}, {waypoint.point.lon:.6f}")

        print_drone_status(state, f"СОСТОЯНИЕ ПЕРЕД ДВИЖЕНИЕМ К {waypoint.name}")

        # Отправляем дрона в точку
        print(f"   Отправка команды перелета...")
        if not goto_position(master, waypoint.point.lat, waypoint.point.lon, waypoint.target_altitude):
            print("   ❌ Не удалось отправить команду")
            continue

        # Отслеживаем движение к точке
        print(f"   Отслеживание движения к точке...")
        half_way_reported = False

        for attempt in range(120):  # Максимум 120 секунд на точку
            current_pos = Point(state.lat_deg, state.lon_deg, state.alt_rel_m)
            distance = haversine_distance(current_pos, waypoint.point)

            # Проверяем половину пути
            if not half_way_reported and waypoint.initial_distance > 0 and distance < waypoint.initial_distance / 2:
                print("\n" + "➖" * 20)
                print(f"🎯 ПОЛОВИНА ПУТИ ДО {waypoint.name}")
                print("➖" * 20)
                print(f"   Пройдено: {format_distance(waypoint.initial_distance - distance)}")
                print(f"   Осталось: {format_distance(distance)}")
                half_way_reported = True

            # Проверяем достижение точки
            if distance < 5.0:  # Достигли точки (в пределах 5 метров)
                print("\n" + "✅" * 20)
                print(f"🎯 ДОСТИГНУТА ТОЧКА: {waypoint.name}")
                print("✅" * 20)
                print(f"   Фактическая высота: {state.alt_rel_m:.1f} м")
                waypoint.reached = True
                break

            # Периодический вывод статуса
            if attempt % 10 == 0:
                print(f"   Расстояние до цели: {format_distance(distance)}, "
                      f"Высота: {state.alt_rel_m:.1f} м, "
                      f"Скорость: {state.ground_speed:.1f} м/с")

            # Проверяем нештатные ситуации
            if not state.armed:
                print("   ❌ Дрон DISARM, прерываем миссию")
                return False

            if state.mode != "GUIDED":
                print(f"   ❌ Режим изменился на {state.mode}, прерываем миссию")
                return False

            time.sleep(1)
        else:
            print(f"   ⚠️  Таймаут достижения точки {waypoint.name}")
            print(f"   Продолжаем к следующей точке...")

    # 5. Возврат домой и посадка
    print("\n" + "🏠" * 30)
    print("🏠 ВОЗВРАТ ДОМОЙ И ПОСАДКА")
    print("🏠" * 30)

    home_waypoint = tracker.waypoints[-1]  # Последняя точка - посадка

    print(f"   Отправка команды посадки...")
    if not land(master):
        print("   ❌ Не удалось выполнить посадку")
        return False

    # Ждем посадки
    print("   Ожидание посадки...")
    for i in range(60):
        if state.alt_rel_m <= 1.0:
            print(f"   ✅ Посадка завершена: высота={state.alt_rel_m:.1f} м")
            home_waypoint.reached = True
            break

        if i % 5 == 0:
            print(f"   Текущая высота: {state.alt_rel_m:.1f} м")

        time.sleep(1)
    else:
        print("   ⚠️  Таймаут посадки")

    print("\n" + "🎉" * 30)
    print("🎉 МИССИЯ ВЫПОЛНЕНА!")
    print("🎉" * 30)

    # Выводим статистику
    reached_count = sum(1 for w in tracker.waypoints if w.reached)
    print(f"   Достигнуто точек: {reached_count}/{len(tracker.waypoints)}")
    print(f"   Общее время полета: {state.flight_time:.0f} с")
    print(f"   Остаток батареи: {state.battery_remaining_pct:.0f}%")

    print_drone_status(state, "ФИНАЛЬНОЕ СОСТОЯНИЕ")

    return True

def main():
    """Основная функция программы"""
    global stop_monitoring, monitor_thread

    # Подключаемся к симулятору
    master = connect_to_simulator()
    if master is None:
        return

    # Инициализируем состояние дрона
    state = DroneState()
    tracker = MissionTracker()

    # Запускаем мониторинг в отдельном потоке
    stop_monitoring = False
    monitor_thread = threading.Thread(
        target=monitor_loop,
        args=(master, state, lambda: stop_monitoring),
        daemon=True
    )
    monitor_thread.start()

    # Даем время мониторингу начать работу
    print("\n⏳ Запуск мониторинга телеметрии...")
    time.sleep(3)

    try:
        # Ждем GPS координаты
        wait_for_gps(state)

        # Настраиваем миссию в трекере
        setup_mission_tracker(state, tracker)

        # Выполняем миссию через GUIDED команды
        execute_mission_guided(state, master, tracker)

        # Ждем перед завершением
        print("\nЗавершение программы через 3 секунды...")
        time.sleep(3)

    except KeyboardInterrupt:
        print("\n⚠️  Программа прервана пользователем")

    except Exception as e:
        print(f"\n❌ Критическая ошибка: {e}")
        import traceback
        traceback.print_exc()

    finally:
        # Останавливаем мониторинг
        print("\nОстановка мониторинга...")
        stop_monitoring = True
        if monitor_thread:
            monitor_thread.join(timeout=2)

        # Закрываем соединение
        if master:
            master.close()

        print("\n✅ Программа завершена!")

if __name__ == "__main__":
    main()
