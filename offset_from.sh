
python -c "
import math
# Haversine constans
RADIUS = 6371008.7714 #mean
CIRCUM = 2*math.pi*RADIUS
METERS_PER_DEG = CIRCUM/360

def offset_position(lat, lon, offset_north, offset_east):
    delta_lat = offset_north / METERS_PER_DEG
    delta_lon = offset_east / (METERS_PER_DEG * math.cos(math.radians(lat)))

    return lat+delta_lat, lon+delta_lon
print(offset_position($1,$2,$3,$4))
    "

