# FT 13/8/26

import numpy as np
from geographiclib.geodesic import Geodesic as geodesic_gglib


@np.vectorize
def _ned_to_xyz_vectorised(n, e, d):
    return e, n, -d

def ned_to_xyz(n, e, d):
    # Call vectorised method
    transformed_coords = _ned_to_xyz_vectorised(n, e, d)
    return np.array(transformed_coords)

def xyz_to_ned(x, y, z):
    # In the current setup, can just reapply the ned_to_xyz transformation to invert.
    return ned_to_xyz(x, y, z)

@np.vectorize(signature='(),(),()->(3,3)')
def calc_body_to_inertial(phi, theta, psi):
    return np.array([
        [np.cos(theta)*np.cos(psi),   np.sin(phi)*np.sin(theta)*np.cos(psi) - np.cos(phi)*np.sin(psi),   np.cos(phi)*np.sin(theta)*np.cos(psi) + np.sin(phi)*np.sin(psi)],
        [np.cos(theta)*np.sin(psi),   np.sin(phi)*np.sin(theta)*np.sin(psi) + np.cos(phi)*np.cos(psi),   np.cos(phi)*np.sin(theta)*np.sin(psi) - np.sin(phi)*np.cos(psi)],
        [-np.sin(theta),               np.sin(phi)*np.cos(theta),                                            np.cos(phi)*np.cos(theta)]
        ])

@np.vectorize(signature='(),()->(3,3)')
def calc_wind_to_body(alpha, beta):
    return np.array([
        [np.cos(alpha)*np.cos(beta),      -np.cos(alpha)*np.sin(beta),      -np.sin(alpha)],
        [np.sin(beta),                     np.cos(beta),                      0],
        [np.sin(alpha)*np.cos(beta),      -np.sin(alpha)*np.sin(beta),      np.cos(alpha)]
        ])

@np.vectorize
def calc_rel_pos_ned(init_lat, init_lon, init_alt, lat, lon, alt):
    lat_dist_m = geodesic_gglib.WGS84.Inverse(init_lat, init_lon, lat, init_lon)['s12']*np.sign(lat - init_lat)
    lon_dist_m = geodesic_gglib.WGS84.Inverse(init_lat, init_lon, init_lat, lon)['s12']*np.sign(lon - init_lon)
    return lat_dist_m, lon_dist_m, -(alt - init_alt)

# Convert waypoints from local coord system to (lat, lon, alt)
# Just using direct vs arc-length distance - is this alright for small distances? (https://geographiclib.sourceforge.io/html/python/code.html#geographiclib.geodesic.Geodesic.ArcDirect)
def ned_to_latlonalt(home_coords, n, e, d):
    azi_deg = -(np.degrees(np.arctan2(n, e)) - 90) % 360
    dist_m = np.sqrt(n**2 + e**2)
    # Calculate azi_deg and dist_m
    res = geodesic_gglib.WGS84.Direct(home_coords[0], home_coords[1], azi_deg, dist_m)
    return res["lat2"], res["lon2"], -d