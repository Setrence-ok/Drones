"""
Модуль мониторинга состояния дрона
"""
from dataclasses import dataclass
import time
from pymavlink import mavutil

@dataclass
class DroneState:
    """Класс для хранения текущего состояния дрона"""
    last_update: float = 0.0
    mode: str = "UNKNOWN"
    armed: bool = False

    # Координаты
    lat_deg: float = 0.0
    lon_deg: float = 0.0
    alt_rel_m: float = 0.0  # Высота ОТНОСИТЕЛЬНАЯ (от точки взлета)
    alt_amsl_m: float = 0.0  # Высота над уровнем моря (абсолютная)

    # Батарея
    battery_voltage_v: float = 0.0
    battery_remaining_pct: float = 0.0

    # Дополнительные параметры
    ground_speed: float = 0.0  # скорость относительно земли, м/с
    heading: float = 0.0       # курс, градусы
    flight_time: float = 0.0   # время полета, секунды

    # Текущая цель миссии
    current_waypoint: int = -1

    # GPS статус
    gps_fix_type: int = 0
    gps_satellites: int = 0
    gps_hdop: float = 99.99

def _handle_heartbeat(master, msg, state: DroneState) -> None:
    """Обработка сообщения HEARTBEAT"""
    # Статус ARM
    state.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

    # Определение режима
    try:
        if hasattr(master, 'mode_mapping'):
            mode_mapping = master.mode_mapping()
            if mode_mapping:
                mode_id = msg.custom_mode
                # Поиск режима по ID
                for name, mid in mode_mapping.items():
                    if mid == mode_id:
                        state.mode = name
                        break
                else:
                    state.mode = f"UNKNOWN({mode_id})"
    except:
        state.mode = "ERROR"

def _handle_global_position_int(msg, state: DroneState) -> None:
    """
    Обработка координат GLOBAL_POSITION_INT
    ВАЖНО: relative_alt - высота относительно домашней точки (в мм)
           alt - высота над уровнем моря (в мм)
    """
    # Координаты в градусах * 10^7
    state.lat_deg = msg.lat / 1e7
    state.lon_deg = msg.lon / 1e7

    # ВАЖНО: Проверяем, что это правильные поля!
    # relative_alt - высота относительно домашней точки в МИЛЛИМЕТРАХ
    # alt - высота над уровнем моря в МИЛЛИМЕТРАХ

    if hasattr(msg, 'relative_alt'):
        state.alt_rel_m = msg.relative_alt / 1000.0  # мм → метры
    if hasattr(msg, 'alt'):
        state.alt_amsl_m = msg.alt / 1000.0  # мм → метры

    if hasattr(msg, 'hdg'):
        state.heading = msg.hdg / 100.0  # Курс в градусах

def _handle_gps_raw_int(msg, state: DroneState) -> None:
    """
    Обработка GPS данных GPS_RAW_INT
    alt - высота над уровнем моря в МИЛЛИМЕТРАХ
    """
    state.lat_deg = msg.lat / 1e7
    state.lon_deg = msg.lon / 1e7
    state.alt_amsl_m = msg.alt / 1000.0  # Высота в метрах над уровнем моря
    state.gps_fix_type = msg.fix_type  # Тип фиксации GPS
    state.gps_satellites = msg.satellites_visible  # Количество спутников
    if hasattr(msg, 'eph'):
        state.gps_hdop = msg.eph / 100.0  # Точность по горизонтали

def _handle_vfr_hud(msg, state: DroneState) -> None:
    """Обработка данных VFR HUD"""
    state.ground_speed = msg.groundspeed
    if hasattr(msg, 'heading'):
        state.heading = msg.heading  # Курс
    # alt в VFR_HUD - это высота над уровнем моря в метрах
    if hasattr(msg, 'alt'):
        state.alt_amsl_m = msg.alt

def _handle_sys_status(msg, state: DroneState) -> None:
    """Обработка состояния системы"""
    if msg.voltage_battery > 0:
        state.battery_voltage_v = msg.voltage_battery / 1000.0
    if msg.battery_remaining >= 0:
        state.battery_remaining_pct = float(msg.battery_remaining)

def _handle_mission_current(msg, state: DroneState) -> None:
    """Обработка текущей точки миссии"""
    state.current_waypoint = msg.seq

def _handle_attitude(msg, state: DroneState) -> None:
    """Обработка ориентации"""
    pass

def monitor_loop(master: mavutil.mavlink_connection,
                state: DroneState,
                stop_flag_getter=lambda: False) -> None:
    """
    Основной цикл мониторинга телеметрии
    """
    start_time = time.time()
    print_time = time.time()

    # Запрашиваем поток данных
    try:
        master.mav.request_data_stream_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            10,  # 10 Hz для тестирования
            1    # Start sending
        )
    except:
        pass

    print(f"[Мониторинг] Запущен, запрошен поток данных")

    while not stop_flag_getter():
        try:
            # Чтение сообщения
            msg = master.recv_match(
                blocking=False,
                timeout=0.1
            )

            if msg is None:
                continue

            msg_type = msg.get_type()
            now = time.time()

            # Обработка различных типов сообщений
            if msg_type == 'HEARTBEAT':
                _handle_heartbeat(master, msg, state)
            elif msg_type == 'GLOBAL_POSITION_INT':
                _handle_global_position_int(msg, state)
            elif msg_type == 'GPS_RAW_INT':
                _handle_gps_raw_int(msg, state)
            elif msg_type == 'SYS_STATUS':
                _handle_sys_status(msg, state)
            elif msg_type == 'VFR_HUD':
                _handle_vfr_hud(msg, state)
            elif msg_type == 'MISSION_CURRENT':
                _handle_mission_current(msg, state)
            elif msg_type == 'ATTITUDE':
                _handle_attitude(msg, state)

            # Обновление времени полета
            state.flight_time = now - start_time
            state.last_update = now

            # Отладочный вывод каждые 3 секунды
            if now - print_time > 3.0:
                print_time = now
                print(f"[Мониторинг] Координаты: {state.lat_deg:.6f}, {state.lon_deg:.6f}")
                print(f"[Мониторинг] Высота REL: {state.alt_rel_m:.1f}м, AMSL: {state.alt_amsl_m:.1f}м")
                print(f"[Мониторинг] Режим: {state.mode}, ARM: {state.armed}, GPS фикс: {state.gps_fix_type}")

        except Exception as e:
            # Не выводим ошибки постоянно, чтобы не засорять вывод
            time.sleep(0.05)