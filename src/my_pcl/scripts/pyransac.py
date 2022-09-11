#!/home/big/anaconda3/bin/python
import sensor_msgs.point_cloud2 as pc2
import rospy
import numpy as np
import ctypes
import struct
import pcl
import sys
sys.path.append(".")
import ros_numpy
import pyransac3d as pyrsc
import open3d as o3d
import open3d_ros_helper as orh
from sensor_msgs.msg import PointCloud2
class pyransac :
        def __init__(self):
            '''initiliaze  ros stuff '''
            self.Subscriber()

        def Subscriber(self):
            self.cloud_sub = rospy.Subscriber("output", PointCloud2,self.ros_to_pcl,queue_size=1, buff_size=52428800)      

        def ros_to_pcl(self,ros_point_cloud):
            pc = ros_numpy.numpify(ros_point_cloud)
            points=np.zeros((pc.shape[0],3))
            points[:,0]=pc['x']
            points[:,1]=pc['y']
            points[:,2]=pc['z']
            p = pcl.PointCloud(np.array(points, dtype=np.int32))
https://github.com/ANYbotics/point_cloud_io
            print(p)
            out_pcd = o3d.geometry.PointCloud()    
            out_pcd.points = o3d.utility.Vector3dVector(p)
            o3d.io.write_point_cloud("/home/big/cloud.ply",out_pcd)
                # self.PlaneDetect(out_pcd)
        def PlaneDetect(self,pcl_data):
            pcd_load = o3d.io.read_point_cloud("/home/big/cloud.ply")
            points = np.asarray(pcl_data.points)
            plano1 = pyrsc.Plane()
            best_eq, best_inliers = plano1.fit(points, thresh=0.1, minPoints=100, maxIteration=50)
            plane = pcd_load.select_by_index(best_inliers).paint_uniform_color([1, 0, 0])
            not_plane = pcd_load.select_by_index(best_inliers, invert=True)
            o3d.visualization.draw_geometries([not_plane])


            

def main(args):
  imu_node = pyransac()
  rospy.init_node('pyransac')
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)