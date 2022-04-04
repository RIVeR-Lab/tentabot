#!/usr/bin/env python3

'''
LAST UPDATE: 2022.02.23

AUTHOR: Hongyu Li (LHY)
        Neset Unver Akmandor (NUA)

E-MAIL: li.hongyu1@northeastern.edu
        akmandor.n@northeastern.edu

DESCRIPTION: TODO...

REFERENCES:
    This function is modifed from Navrep project (https://github.com/ethz-asl/navrep)
'''

import numpy as np
from matplotlib import pyplot as plt
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
import math
import cmath

def fast_lidar_to_rings(scans, angle_levels, range_levels, range_level_mins, range_level_maxs):
    rings = np.zeros(
        (scans.shape[0], angle_levels, range_levels, 1), dtype=np.uint8
    )
    clidar_to_rings(scans, angle_levels, range_levels, range_level_mins, range_level_maxs, rings)
    return rings

'''
# cdef clidar_to_rings():
def clidar_to_rings(
    scans,
    angle_levels,
    range_levels,
    range_level_mins,
    range_level_maxs,
    rings,
):
    """
    scans: ndarray (n_scans, n_rays)   0-100 [m]
    rings: ndarray (n_scans, angle_levels, range_levels, n_channels)
    """
    n_scans = scans.shape[0]
    n_rays = scans.shape[1]
    CHANNEL = 0
    downsample_factor = n_rays * 1.0 / angle_levels  # >= 1
    # vars
    ray_idx_start = 0
    ray_idx_end = 0
    angle_idx = 0
    range_idx = 0
    scan_idx = 0
    ray_idx = 0
    r_min = 0.0
    r_max = 0.0
    dist = 0.0
    cell_value = 0
    for angle_idx in range(angle_levels):
        # generate ray indices corresponding to current angle level
        ray_idx_start = int(math.ceil(angle_idx * downsample_factor))
        ray_idx_end = int(math.ceil((angle_idx + 1) * downsample_factor))
        for range_idx in range(range_levels):
            r_min = range_level_mins[range_idx]
            r_max = range_level_maxs[range_idx]
            for scan_idx in range(n_scans):
                # initialize value of cell at (angle_idx, range_idx)
                cell_value = 0
                # check if any relevant rays hit cell or fall short
                for ray_idx in range(ray_idx_start, ray_idx_end):
                    dist = scans[scan_idx, ray_idx]
                    if dist == 0:
                        continue
                    # if a ray hits the cell the value is 2
                    if r_min <= dist and dist < r_max:
                        cell_value = 2
                        break
                    # if a ray falls short of the cell the value is at least 1
                    if dist < r_min:
                        cell_value = 1
                        continue
                rings[scan_idx, angle_idx, range_idx, CHANNEL] = cell_value
    rings = np.zeros((scans.shape[0], angle_levels, range_levels, 1), dtype=np.uint8)
    clidar_to_rings(scans, angle_levels, range_levels, range_level_mins, range_level_maxs, rings)
    return rings
'''

'''
DESCRIPTION: TODO...
'''
def clidar_to_rings(
    scans,
    angle_levels,
    range_levels,
    range_level_mins,
    range_level_maxs,
    rings,
):
    """
    scans: ndarray (n_scans, n_rays)   0-100 [m]
    rings: ndarray (n_scans, angle_levels, range_levels, n_channels)
    """
    n_scans = scans.shape[0]
    n_rays = scans.shape[1]
    CHANNEL = 0
    downsample_factor = n_rays * 1.0 / angle_levels  # >= 1
    # vars
    ray_idx_start = 0
    ray_idx_end = 0
    angle_idx = 0
    range_idx = 0
    scan_idx = 0
    ray_idx = 0
    r_min = 0.0
    r_max = 0.0
    dist = 0.0
    cell_value = 0
    for angle_idx in range(angle_levels):
        # generate ray indices corresponding to current angle level
        ray_idx_start = int(math.ceil(angle_idx * downsample_factor))
        ray_idx_end = int(math.ceil((angle_idx + 1) * downsample_factor))
        for range_idx in range(range_levels):
            r_min = range_level_mins[range_idx]
            r_max = range_level_maxs[range_idx]
            for scan_idx in range(n_scans):
                # initialize value of cell at (angle_idx, range_idx)
                cell_value = 0
                # check if any relevant rays hit cell or fall short
                for ray_idx in range(ray_idx_start, ray_idx_end):
                    dist = scans[scan_idx, ray_idx]
                    if dist == 0:
                        continue
                    # if a ray hits the cell the value is 2
                    if r_min <= dist and dist < r_max:
                        cell_value = 2
                        break
                    # if a ray falls short of the cell the value is at least 1
                    if dist < r_min:
                        cell_value = 1
                        continue
                rings[scan_idx, angle_idx, range_idx, CHANNEL] = cell_value

'''
DESCRIPTION: TODO...
'''
def generate_rings(angle_levels=64,
                    range_levels=64,
                    expansion_term=3.12,
                    min_resolution=0.01,
                    min_dist=0.15,
                    max_dist=12.0,
                    VISUALIZE=False):

    x = np.linspace(0, 1, range_levels - 2)
    expansion_curve = np.power(x, expansion_term)  # a.k.a range_level_depths
    renormalisation_factor = np.sum(expansion_curve) / (
        max_dist - (min_resolution * (range_levels - 2)) - min_dist
    )
    range_level_depths = expansion_curve / renormalisation_factor + min_resolution
    range_level_maxs = np.cumsum(range_level_depths) + min_dist
    range_level_maxs = np.concatenate([[min_dist], range_level_maxs, [np.inf]]).astype(np.float32)
    range_level_mins = np.concatenate([[0.0], range_level_maxs[:-1]]).astype(np.float32)

    if VISUALIZE:

        th = np.linspace(-7.0 / 8.0 * np.pi, 7.0 / 8.0 * np.pi, angle_levels)

        plt.figure("curve")
        plt.plot(range_level_maxs[:-1])
        for x, y in enumerate(range_level_maxs[:-1]):
            plt.axhline(y)
            plt.axvline(x)

        plt.figure("rings")
        for i, r in enumerate(range_level_maxs[:-1]):
            plt.gca().add_artist(plt.Circle((0, 0), r, color="k", fill=False))

        for i in range(angle_levels):
            plt.plot([min_dist * np.cos(th), r * np.cos(th)], [min_dist * np.sin(th), r * np.sin(th)], "k",)
            plt.axis("equal")

        plt.gca().add_artist(plt.Circle((1, 1), 0.3, color="r", zorder=3))
        plt.tight_layout()

    '''
    DESCRIPTION: TODO...scans: ndarray (n, N_RAYS)   0-100 [m]
        rings: ndarray (n_scans, angle_levels, range_levels, n_channels)
    '''
    def lidar_to_rings(scans):
        return fast_lidar_to_rings(scans, angle_levels, range_levels, range_level_mins, range_level_maxs)

    '''
    DESCRIPTION: TODO...
    '''
    def rings_to_lidar(rings, N_RAYS=1080):

        CHANNEL = 0
        i_to_j, j_to_ii = generate_downsampling_map(N_RAYS, angle_levels)
        scans = np.zeros((rings.shape[0], N_RAYS), dtype=np.float32)
        for j, ii in enumerate(j_to_ii):
            scans[:, ii] = range_level_mins[np.argmax(rings[:, j, :, :] > 0.4, axis=1)][
                :, None, CHANNEL
            ]
        return scans

    '''
    DESCRIPTION: TODO...
    '''
    def visualize_rings(ring, 
                        scan=None, 
                        angle_min=-0.9, 
                        angle_max=5.0,
                        range_max=5.0, 
                        fig=None, 
                        ax=None, 
                        plot_regen=False):
        
        CHANNEL = 0
        th = np.linspace(angle_min, angle_max, angle_levels)
        r = range_level_mins
        thth, rr = np.meshgrid(th, r)
        
        if fig is None:
            fig = plt.figure("rings")
        
        if ax is None:
            ax = fig.add_subplot(111, projection="polar")
        
        ax.clear()
        ax.pcolormesh(thth, rr, ring[:, :, CHANNEL].T, cmap=plt.cm.Greys)
        
        if scan is not None:
            scan_regen = rings_to_lidar(ring[None, :, :, :], scan.shape[0])[0, :]
            scan_th = np.linspace(angle_min, angle_max, scan.shape[0])
            
            plt.plot(scan_th, scan, "r")

            if scan[0] > range_max:
                scan[0] = range_max

            '''
            polar_x = np.array([cmath.rect(scan_th[0], scan[0]).real])
            polar_y = np.array([cmath.rect(scan_th[0], scan[0]).imag])
            for i in range(1, len(scan_th)):

                if scan[i] > range_max:
                    scan[i] = range_max

                tmp_x = np.array([cmath.rect(scan_th[i], scan[i]).real])
                tmp_y = np.array([cmath.rect(scan_th[i], scan[i]).imag])

                polar_x = np.concatenate((polar_x,tmp_x))
                polar_y = np.concatenate((polar_y,tmp_y))
            '''
            if plot_regen:
                plt.plot(scan_th, scan_regen, "g")

            plt.show()

            '''
            plt.figure()
            plt.plot(polar_x, polar_y)
            plt.show()
            '''

        return ax

    return {"range_level_mins": range_level_mins,
            "range_level_maxs": range_level_maxs,
            "lidar_to_rings": lidar_to_rings,
            "rings_to_lidar": rings_to_lidar,
            "visualize_rings": visualize_rings,
            "rings_to_bool": 2.0}

'''
DESCRIPTION: TODO...
'''
def generate_downsampling_map(I, J):

    downsample_factor = I * 1.0 / J
    i_to_j = np.floor(np.arange(I) / downsample_factor).astype(int)
    j_to_ii = [
        np.arange(
            np.ceil(j * downsample_factor),
            np.ceil((j + 1) * downsample_factor),
            dtype=int,
        )
        for j in range(J)
    ]
    return i_to_j, j_to_ii

'''
DESCRIPTION: TODO...
'''
class RingsGenerator():

    '''
    DESCRIPTION: TODO...
    '''
    def __init__(self, laser_rings_msg="laser_rings", 
                        rings_angle_level=64, 
                        rings_range_level=64, 
                        rings_resolution=0.01) -> None:

        self.data = None
        self.ring_def = None
        self.rings_angle_level = rings_angle_level
        self.rings_range_level = rings_range_level
        self.rings_resolution = rings_resolution

        self.pub = rospy.Publisher(laser_rings_msg, Float32MultiArray, queue_size=10)
        
        print("laser_to_rings::RingsGenerator -> Ready to publish laser rings...")
        pass

    '''
    DESCRIPTION: TODO...
    '''
    def laser_callback(self, data: LaserScan):
        
        self.data = data
        
        if self.ring_def is None:
            self.ring_def = generate_rings(angle_levels=self.rings_angle_level,
                                            range_levels=self.rings_range_level,
                                            min_resolution=self.rings_resolution,
                                            min_dist=data.range_min,
                                            max_dist=data.range_max,
                                            VISUALIZE=False)
        
        scan_data = data.ranges
        scan_data = np.array(scan_data).reshape(1, len(scan_data))
        rings = self.ring_def["lidar_to_rings"](scan_data.astype(np.float32))
        rings_image = rings[-1,:,:,-1]
        rings_scale = 1 / np.max(rings_image)
        rings_image_scaled = rings_scale * rings_image
        rings = rings_image_scaled.reshape(self.rings_angle_level * self.rings_range_level)
        
        rings_msg = Float32MultiArray()
        rings_msg.data = rings
        self.pub.publish(rings_msg)

if __name__ == "__main__":

    rospy.init_node('laser_to_rings')

    laser_scan_msg = rospy.get_param('laser_scan_msg', "")
    laser_rings_msg = rospy.get_param('laser_rings_msg', "")
    rings_angle_level = rospy.get_param('rings_angle_level', 0)
    rings_range_level = rospy.get_param('rings_range_level', 0)
    rings_resolution = rospy.get_param('rings_resolution', 0.0)

    ringo = RingsGenerator(laser_rings_msg=laser_rings_msg, 
                            rings_angle_level=rings_angle_level, 
                            rings_range_level=rings_range_level, 
                            rings_resolution=rings_resolution)

    rospy.Subscriber(laser_scan_msg, LaserScan, ringo.laser_callback)

    '''
    # DEBUG:
    rospy.sleep(0.5)
    ring_def = generate_rings(min_dist=ringo.data.range_min, max_dist=ringo.data.range_max)
    scan_data = ringo.data.ranges
    scan_data = np.array(scan_data).reshape(1,-1)
    rings = ring_def["lidar_to_rings"](scan_data.astype(np.float32))
    ringo = 0.5 * rings[0, :, :, -1]
    print(ringo[0,:])
    plt.figure()
    plt.imshow(ringo)
    ring_def["visualize_rings"](rings[0, :, :, :], scan=scan_data[0], angle_min=-0.9, angle_max=5.0, range_max=3.5)
    '''

    rospy.spin()