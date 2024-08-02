import struct

from sensor_msgs.msg import PointCloud2, Image
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Header

endian = "<>"

def get_xyz_from_point_xy(point_list: PointCloud2, x: int, y: int):
    point = x * point_list.point_step + y * point_list.row_step

    x_pos = point + point_list.fields[0].offset
    y_pos = point + point_list.fields[1].offset
    z_pos = point + point_list.fields[2].offset

    x = (point_list.data[x_pos + 3] << 8*3) + (point_list.data[x_pos + 2] << 8*2) + (point_list.data[x_pos + 1] << 8) + point_list.data[x_pos]
    y = (point_list.data[y_pos + 3] << 8*3) + (point_list.data[y_pos + 2] << 8*2) + (point_list.data[y_pos + 1] << 8) + point_list.data[y_pos]
    z = (point_list.data[z_pos + 3] << 8*3) + (point_list.data[z_pos + 2] << 8*2) + (point_list.data[z_pos + 1] << 8) + point_list.data[z_pos]

    x_float = struct.unpack(f'{endian[point_list.is_bigendian]}f', struct.pack('I', x))[0]
    y_float = struct.unpack(f'{endian[point_list.is_bigendian]}f', struct.pack('I', y))[0]
    z_float = struct.unpack(f'{endian[point_list.is_bigendian]}f', struct.pack('I', z))[0]

    return (x_float, y_float, z_float)

def get_depth_from_xy(point_list: Image, x: int, y: int):
    point = x + y * point_list.step

    depth = (point_list.data[point+3] << 8*3)
    depth += (point_list.data[point+2] << 8*2)
    depth += (point_list.data[point+1] << 8)
    depth += (point_list.data[point])

    depth_float = struct.unpack(f'{endian[point_list.is_bigendian]}f', struct.pack('I', depth))[0]

    return depth_float

def get_tf_from_xyz(xyz: tuple, header: Header, child_frame: str):
    x_float, y_float, z_float = xyz

    t = TransformStamped()
    t.header = header
    t.child_frame_id = child_frame

    t.transform.translation.x = x_float
    t.transform.translation.y = y_float
    t.transform.translation.z = z_float

    return t
