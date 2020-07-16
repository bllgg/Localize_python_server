def rssi_to_dist(tx_power, rssi, n):
    distance = 10 ** ((tx_power - rssi) / (10 * n))
    return distance