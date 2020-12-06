
PI = 3.1415
def deg2rad(deg):
    """Convert [0-360]deg to [-pi,pi]rad => [180,360]deg ~ [-pi,0]rad  ,  [0,180]deg ~ [0,pi]rad
    
    Input:
    ------
    deg: int
    """
    if deg in range(0,181):
        return (deg * PI) / 180 
    if deg in range(181,361):
        return ((deg - 360) * PI) / 180

def rad2deg(rad):
    """Convert yaw in [-pi,pi]rad to [0,360]deg, 0rad = 0deg, 
    [-pi,0]rad~[180,360] 
    [0,pi]rad~[0,180]

    Input:
    ------
    rad : radian in range [-pi,pi]
    """
    if rad is None:
        pass
    elif rad >= 0:
        return (rad * 180) / PI
    else :
        return 360 - (-rad * 180) / PI

print(rad2deg(deg2rad(40)))
printgt
