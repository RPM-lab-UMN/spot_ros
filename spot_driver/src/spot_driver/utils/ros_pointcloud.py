from sensor_msgs.msg import PointCloud2, PointField, Image
from sensor_msgs import point_cloud2
import numpy as np
import rospy
import struct
from std_msgs.msg import Header

def ros_image_to_rgb_ndarray(image: Image) -> np.ndarray:
    H, W = image.height, image.width
    if image.encoding == 'rgb8':
        rgb = np.frombuffer(image.data, dtype=np.byte)
        rgb = rgb.reshape(H, W, 3).astype(np.uint8)
    elif image.encoding == 'rgba8':
        rgb = np.frombuffer(image.data, dtype=np.byte)
        rgb = rgb.reshape(H, W, 4).astype(np.uint8)
    elif image.encoding == 'bgra8':
        rgb = np.frombuffer(image.data, dtype=np.byte)
        rgb = rgb.reshape(H, W, 4)[:, :, (2,0,1)].astype(np.uint8)
    else: 
        raise RuntimeError(f'Not setup to handle {image.encoding} ros image as rgb ndarray.')
    return rgb


def ros_image_to_depth_ndarray(image: Image) -> np.ndarray:
    H, W = image.height, image.width
    if image.encoding == '16UC1':
        d = np.frombuffer(image.data, dtype=np.uint16).reshape(H, W)
    elif image.encoding == 'bgra8':
        rgbd = np.frombuffer(image.data, dtype=np.byte)
        rgbd = rgbd.reshape(H, W, 4)[:, :, (2,0,1)].astype(np.uint8)
        d = rgbd[:,:,3].astype(np.uint16)
    return d.astype(np.float32) / 1000


def _get_xyz(d, K):
    W,H = d.shape[1], d.shape[0]
    vu = np.mgrid[:H, :W]
    ones = np.ones((1,H,W))
    uv1 = np.concatenate([vu[[1]], vu[[0]], ones], axis=0)
    uv1_prime = uv1 * d                                # [3, H, W]
    xyz = np.linalg.inv(K) @ uv1_prime.reshape(3, -1)  # [3,  HxW]
    return xyz


def images_to_pointcloud2(color:Image, depth:Image, K) -> PointCloud2:
    assert(color.height == depth.height and color.width == depth.width)
    if (len(K) > 4): K = np.array(K).reshape(3,3)
    rgb = ros_image_to_rgb_ndarray(color)
    d = ros_image_to_depth_ndarray(depth)
    xyz = _get_xyz(d, K).T                              # [HxW, 3]
    rgb = rgb.reshape(-1, 3)                            # [HxW, 3]
    header = color.header
    header.stamp = rospy.Time.now()
    return package_ros_pointcloud2(xyz, rgb, header)



def package_ros_pointcloud2(xyz, rgb, header):
    '''Given lists of xyz data and associated rgb returns pointcloud2 message'''
    fields = [PointField('x',   0,  PointField.FLOAT32, 1), 
              PointField('y',   4,  PointField.FLOAT32, 1), 
              PointField('z',   8,  PointField.FLOAT32, 1)
              ,PointField('rgba', 12, PointField.UINT32,  1)]

    a = np.ones((rgb.shape[0], 1)) * 255
    rgba = np.concatenate([rgb, a], axis=1)
    rgba = rgba.astype(np.uint8)
    xyz = xyz.astype(np.float32)

    points = []
    for i in range(len(xyz)):
        b, g, r, a = tuple(rgba[i])
        x, y, z = tuple(xyz[i])
        color = struct.unpack('I', struct.pack('BBBB', r, g, b, a))[0]
        pt = [x, y, z, color]
        points.append(pt)
    cloud = point_cloud2.create_cloud(header, fields, points)
    cloud.is_dense = True
    return cloud

def create_waypoint_pointcloud_message(xyz):
    rgb = np.array([[0, 0, 255]])
    
    rgb = np.repeat(rgb, xyz.shape[0], axis=0)
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "odom"
    return package_ros_pointcloud2(xyz, rgb, header)