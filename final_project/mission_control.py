"""
Модуль управления миссиями (полётными заданиями)
"""
import time
from dataclasses import dataclass
from typing import List, Optional
from pymavlink import mavutil

@dataclass
class MissionItem:
    """
    Один пункт миссии в формате MISSION_ITEM_INT
    """
    seq: int
    frame: int
    command: int
    current: int
    autocontinue: int
    param1: float
    param2: float
    param3: float
    param4: float
    x: int  # lat * 1e7
    y: int  # lon * 1e7
    z: float  # alt (м)

def clear_mission(master: mavutil.mavlink_connection) -> bool:
    """Очистка текущей миссии"""
    try:
        print("   Отправка MISSION_CLEAR_ALL...")
        master.mav.mission_clear_all_send(
            master.target_system,
            master.target_component
        )

        # Ждем подтверждения
        msg = master.recv_match(type=['MISSION_ACK'], blocking=True, timeout=3)
        if msg:
            if msg.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
                print("   ✅ Миссия очищена")
                return True
            else:
                print(f"   ❌ Ошибка очистки миссии: {msg.type}")
                return False
        else:
            print("   ❌ Таймаут очистки миссии")
            return False

    except Exception as e:
        print(f"   ❌ Ошибка при очистке миссии: {e}")
        return False

def upload_mission(master: mavutil.mavlink_connection, items: List[MissionItem]) -> bool:
    """Загрузка миссии с улучшенной обработкой ошибок"""
    if not items:
        print("   ❌ Список миссий пуст")
        return False

    print(f"   Начинаем загрузку {len(items)} точек миссии...")

    try:
        # 1. Отправляем количество точек
        count = len(items)
        print(f"   Отправляем MISSION_COUNT: {count} точек")
        master.mav.mission_count_send(
            master.target_system,
            master.target_component,
            count,
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION
        )

        # 2. Ожидаем запросы на каждую точку
        sent = 0
        timeout = time.time() + 30  # 30 секунд на всю загрузку

        while sent < count and time.time() < timeout:
            # Ждем запрос на конкретную точку
            msg = master.recv_match(
                type=['MISSION_REQUEST_INT', 'MISSION_REQUEST', 'MISSION_ACK'],
                blocking=True,
                timeout=5
            )

            if msg is None:
                print(f"   ⏰ Таймаут ожидания запроса для точки {sent}")
                continue

            msg_type = msg.get_type()

            if msg_type == 'MISSION_REQUEST_INT' or msg_type == 'MISSION_REQUEST':
                # Определяем номер запрошенной точки
                if msg_type == 'MISSION_REQUEST_INT':
                    seq = msg.seq
                else:  # MISSION_REQUEST
                    seq = msg.seq

                print(f"   Получен запрос на точку {seq}")

                if seq < 0 or seq >= count:
                    print(f"   ❌ Некорректный номер точки: {seq}")
                    continue

                # Отправляем запрошенную точку
                item = items[seq]

                if msg_type == 'MISSION_REQUEST_INT':
                    master.mav.mission_item_int_send(
                        master.target_system,
                        master.target_component,
                        item.seq,
                        item.frame,
                        item.command,
                        item.current,
                        item.autocontinue,
                        item.param1,
                        item.param2,
                        item.param3,
                        item.param4,
                        item.x,
                        item.y,
                        item.z,
                        mavutil.mavlink.MAV_MISSION_TYPE_MISSION
                    )
                else:
                    # Используем старый формат MISSION_ITEM
                    master.mav.mission_item_send(
                        master.target_system,
                        master.target_component,
                        item.seq,
                        item.frame,
                        item.command,
                        item.current,
                        item.autocontinue,
                        item.param1,
                        item.param2,
                        item.param3,
                        item.param4,
                        item.x / 1e7,  # Преобразуем обратно в градусы
                        item.y / 1e7,  # Преобразуем обратно в градусы
                        item.z
                    )

                sent += 1
                print(f"   Отправлена точка {seq} ({sent}/{count})")

            elif msg_type == 'MISSION_ACK':
                if msg.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
                    print("   ✅ Миссия успешно загружена")
                    return True
                else:
                    print(f"   ❌ Ошибка загрузки миссии: {msg.type}")
                    return False

        # 3. После отправки всех точек ждем окончательного подтверждения
        print("   Все точки отправлены, ожидаем подтверждения...")
        msg = master.recv_match(type='MISSION_ACK', blocking=True, timeout=5)
        if msg and msg.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
            print("   ✅ Миссия успешно загружена и подтверждена")
            return True
        else:
            print("   ⚠️  Не получено подтверждение загрузки")
            return False

    except Exception as e:
        print(f"   ❌ Критическая ошибка при загрузке миссии: {e}")
        import traceback
        traceback.print_exc()
        return False

def create_simple_test_mission(home_lat: float, home_lon: float) -> List[MissionItem]:
    """
    Создание ПРОСТОЙ тестовой миссии (только взлет и посадка)
    """
    items = []

    print(f"   Создание миссии для координат: {home_lat:.6f}, {home_lon:.6f}")

    # Точка 0: Взлет (20 метров) - используем команду взлета
    items.append(MissionItem(
        seq=0,
        frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        command=mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        current=1,  # Текущая точка
        autocontinue=1,
        param1=0,  # Минимальный pitch
        param2=0,  # Пусто
        param3=0,  # Пусто
        param4=0,  # Yaw angle
        x=int(home_lat * 1e7),  # Широта
        y=int(home_lon * 1e7),  # Долгота
        z=20.0  # Высота взлета
    ))

    # Точка 1: Возврат домой и посадка
    items.append(MissionItem(
        seq=1,
        frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        command=mavutil.mavlink.MAV_CMD_NAV_LAND,
        current=0,
        autocontinue=1,
        param1=0, param2=0, param3=0, param4=0,
        x=int(home_lat * 1e7),
        y=int(home_lon * 1e7),
        z=0.0
    ))

    print(f"   Создано {len(items)} точек миссии")
    return items