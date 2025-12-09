"""
Модуль для отслеживания выполнения миссии
"""
import time
from typing import List, Dict, Optional
from dataclasses import dataclass
from calculations import Point, haversine_distance, format_distance

@dataclass
class MissionWaypoint:
    """Точка миссии с дополнительной информацией"""
    point: Point
    name: str
    target_altitude: float
    reached: bool = False
    half_way_notified: bool = False
    initial_distance: float = 0.0

class MissionTracker:
    """Класс для отслеживания выполнения миссии"""

    def __init__(self):
        self.waypoints: List[MissionWaypoint] = []
        self.current_waypoint_index = 0
        self.start_time = 0
        self.mission_started = False

    def add_waypoint(self, lat: float, lon: float, alt: float, name: str = ""):
        """Добавление точки миссии"""
        point = Point(lat=lat, lon=lon, alt=alt)
        waypoint = MissionWaypoint(
            point=point,
            name=name,
            target_altitude=alt
        )
        self.waypoints.append(waypoint)

    def start_mission(self):
        """Начало выполнения миссии"""
        self.start_time = time.time()
        self.mission_started = True
        self.current_waypoint_index = 0
        print("=" * 60)
        print("🚀 МИССИЯ НАЧАТА!")
        print("=" * 60)

    def get_current_target(self) -> Optional[MissionWaypoint]:
        """Получение текущей цели"""
        if self.current_waypoint_index < len(self.waypoints):
            return self.waypoints[self.current_waypoint_index]
        return None

    def update_position(self, current_pos: Point) -> Dict:
        """
        Обновление текущей позиции и проверка событий

        Возвращает словарь с событиями:
        - 'approaching': приближение к точке (перед началом движения)
        - 'half_way': достигнута половина пути
        - 'reached_target': достигнута целевая точка
        """
        events = {}

        if not self.mission_started or self.current_waypoint_index >= len(self.waypoints):
            return events

        current_target = self.get_current_target()

        # Расчет расстояния до текущей цели
        distance_to_target = haversine_distance(current_pos, current_target.point)

        # Если это начало движения к точке
        if not current_target.reached and distance_to_target > 10:
            if current_target.initial_distance == 0:
                current_target.initial_distance = distance_to_target

                events['approaching'] = {
                    'point_name': current_target.name,
                    'distance': distance_to_target,
                    'point_index': self.current_waypoint_index
                }

        # Проверка достижения половины пути
        half_way_distance = current_target.initial_distance / 2
        if (not current_target.half_way_notified and
            current_target.initial_distance > 0 and
            distance_to_target <= half_way_distance):

            current_target.half_way_notified = True
            events['half_way'] = {
                'point_name': current_target.name,
                'distance_traveled': current_target.initial_distance - distance_to_target,
                'distance_remaining': distance_to_target,
                'point_index': self.current_waypoint_index
            }

        # Проверка достижения цели (в пределах 5 метров)
        if distance_to_target < 5.0 and not current_target.reached:
            current_target.reached = True
            events['reached_target'] = {
                'point_name': current_target.name,
                'point_index': self.current_waypoint_index,
                'mission_time': time.time() - self.start_time
            }

            # Переход к следующей точке
            if self.current_waypoint_index < len(self.waypoints) - 1:
                self.current_waypoint_index += 1
                print(f"\n🔄 Переход к следующей точке: {self.current_waypoint_index + 1}/{len(self.waypoints)}")

        return events

    def get_mission_progress(self) -> float:
        """Возвращает процент выполнения миссии"""
        if not self.waypoints:
            return 0.0

        reached_count = sum(1 for w in self.waypoints if w.reached)
        return (reached_count / len(self.waypoints)) * 100

    def is_mission_complete(self) -> bool:
        """Проверка завершения миссии"""
        return all(w.reached for w in self.waypoints)

    def get_next_target_info(self, current_pos: Point) -> Optional[Dict]:
        """Информация о следующей цели"""
        if self.current_waypoint_index >= len(self.waypoints):
            return None

        target = self.waypoints[self.current_waypoint_index]
        distance = haversine_distance(current_pos, target.point)

        return {
            'name': target.name,
            'distance': distance,
            'formatted_distance': format_distance(distance),
            'target_altitude': target.target_altitude,
            'point_index': self.current_waypoint_index,
            'total_points': len(self.waypoints)
        }
