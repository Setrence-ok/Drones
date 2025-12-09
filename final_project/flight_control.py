"""
Модуль управления полетом дрона
"""
import time
from pymavlink import mavutil

def set_mode(master: mavutil.mavlink_connection, mode_name: str, timeout: float = 5.0) -> bool:
    """
    Универсальная функция смены режима полёта
    Возвращает True при успешной смене режима
    """
    try:
        # Получаем таблицу режимов от автопилота
        mode_mapping = master.mode_mapping()
        if mode_mapping is None or mode_name not in mode_mapping:
            print(f"❌ Режим {mode_name} недоступен")
            return False

        mode_id = mode_mapping[mode_name]

        # Отправляем команду смены режима
        master.mav.set_mode_send(
            master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )

        # Ждем подтверждения
        end_time = time.time() + timeout

        while time.time() < end_time:
            msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=0.5)
            if msg is None:
                continue

            if hasattr(msg, 'custom_mode') and msg.custom_mode == mode_id:
                return True

            time.sleep(0.1)

        return False

    except Exception as e:
        print(f"❌ Ошибка при смене режима: {e}")
        return False

def set_mode_guided(master: mavutil.mavlink_connection) -> bool:
    """Переводит дрон в режим GUIDED"""
    return set_mode(master, "GUIDED")

def set_mode_auto(master: mavutil.mavlink_connection) -> bool:
    """Переводит дрон в режим AUTO"""
    return set_mode(master, "AUTO")

def arm(master: mavutil.mavlink_connection, force: bool = False) -> bool:
    """ARM двигателей"""
    try:
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,  # param1: 1 = arm, 0 = disarm
            0 if not force else 21196,
            0, 0, 0, 0, 0
        )
        return True

    except Exception as e:
        print(f"❌ Ошибка при ARM: {e}")
        return False

def disarm(master: mavutil.mavlink_connection, force: bool = False) -> bool:
    """DISARM двигателей"""
    try:
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0,  # disarm
            0 if not force else 21196,
            0, 0, 0, 0, 0
        )
        return True

    except Exception as e:
        print(f"❌ Ошибка при DISARM: {e}")
        return False

def takeoff(master: mavutil.mavlink_connection, alt_m: float) -> bool:
    """Взлет на указанную высоту"""
    try:
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0, 0, 0, 0,
            0, 0, alt_m
        )
        return True

    except Exception as e:
        print(f"❌ Ошибка при взлете: {e}")
        return False

def land(master: mavutil.mavlink_connection) -> bool:
    """Посадка в текущей точке"""
    try:
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0,
            0, 0, 0, 0,
            0, 0, 0
        )
        return True

    except Exception as e:
        print(f"❌ Ошибка при посадке: {e}")
        return False

def goto_position(master: mavutil.mavlink_connection, lat: float, lon: float, alt: float) -> bool:
    """Отправка дрона в указанную позицию (режим GUIDED)"""
    try:
        # Используем команду NAV_WAYPOINT для режима GUIDED
        master.mav.mission_item_send(
            master.target_system,
            master.target_component,
            0,  # seq
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            2,  # current - 2 означает guided команду
            0,  # autocontinue
            0, 0, 0, 0,  # params
            lat, lon, alt
        )
        print(f"   Команда перелета отправлена: {lat:.6f}, {lon:.6f}, alt={alt:.1f}м")
        return True

    except Exception as e:
        print(f"   ❌ Ошибка при отправке команды перелета: {e}")
        return False

def get_available_modes(master: mavutil.mavlink_connection) -> dict:
    """Возвращает словарь доступных режимов полета"""
    try:
        mode_mapping = master.mode_mapping()
        if mode_mapping:
            return mode_mapping
        else:
            return {}
    except Exception as e:
        print(f"❌ Ошибка при получении режимов: {e}")
        return {}