
import math

def haversine_distance(coord1, coord2):
  R = 6371.0  # Radius of the Earth in kilometers

  lat1, lon1 = math.radians(coord1[0]), math.radians(coord1[1])
  lat2, lon2 = math.radians(coord2[0]), math.radians(coord2[1])

  dlat = lat2 - lat1
  dlon = lon2 - lon1

  a = math.sin(dlat / 2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2)**2
  c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

  distance = R * c
  return distance*1000

def gps_midpoint(coord_list):
  latitudes = [coord[0] for coord in coord_list]
  longitudes = [coord[1] for coord in coord_list]

  avg_latitude = sum(latitudes) / len(coord_list)
  avg_longitude = sum(longitudes) / len(coord_list)

  return [avg_latitude, avg_longitude]

