import rssi_to_distance as rs_to_dist

def trackLocation(x1, y1, x2, y2, x3, y3, rs1, rs2, rs3):
    r1 = rs_to_dist.rssi_to_dist(0, rs1, 2)
    r2 = rs_to_dist.rssi_to_dist(0, rs2, 2)
    r3 = rs_to_dist.rssi_to_dist(0, rs3, 2)

    A = 2*x2 - 2*x1
    B = 2*y2 - 2*y1
    C = r1**2 - r2**2 - x1**2 + x2**2 - y1**2 + y2**2
    D = 2*x3 - 2*x2
    E = 2*y3 - 2*y2

    F = r2**2 - r3**2 - x2**2 + x3**2 - y2**2 + y3**2
    x = (C*E - F*B) / (E*A - B*D)
    y = (C*D - A*F) / (B*D - A*E)
    return x,y


