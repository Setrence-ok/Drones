"""
Модуль для математических расчетов расстояний между точками
"""
import math
from dataclasses import dataclass

@dataclass
class Point:
    """Класс для представления точки с координатами и высотой"""
    lat: float  # широта в градусах
    lon: float  # долгота в градусах
    alt: float  # высота в метрах

def haversine_distance(point1: Point, point2: Point) -> float:
    """
    Вычисляет расстояние между двумя точками на сфере (в метрах)
    по формуле гаверсинусов
    """
    # Радиус Земли в метрах
    R = 6371000.0

    # Преобразование градусов в радианы
    lat1 = math.radians(point1.lat)
    lon1 = math.radians(point1.lon)
    lat2 = math.radians(point2.lat)
    lon2 = math.radians(point2.lon)

    # Разницы координат
    dlat = lat2 - lat1
    dlon = lon2 - lon1

    # Формула гаверсинусов
    a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))

    # Горизонтальное расстояние
    horizontal_distance = R * c

    # Вертикальное расстояние (разница высот)
    vertical_distance = abs(point2.alt - point1.alt)

    # Общее расстояние (евклидово в 3D)
    total_distance = math.sqrt(horizontal_distance**2 + vertical_distance**2)

    return total_distance

def calculate_midpoint(point1: Point, point2: Point) -> Point:
    """
    Вычисляет среднюю точку между двумя точками
    """
    mid_lat = (point1.lat + point2.lat) / 2
    mid_lon = (point1.lon + point2.lon) / 2
    mid_alt = (point1.alt + point2.alt) / 2

    return Point(lat=mid_lat, lon=mid_lon, alt=mid_alt)

def format_distance(distance_m: float) -> str:
    """
    Форматирует расстояние для вывода
    """
    if distance_m < 1000:
        return f"{distance_m:.1f} м"
    else:
        return f"{distance_m/1000:.2f} км"