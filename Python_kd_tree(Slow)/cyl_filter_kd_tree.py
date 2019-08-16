import numpy as np
import math
import operator
from scipy.spatial import cKDTree
import argparse
import open3d as o3d
import time

# take first element for sort
def takefirst(elem):
    return elem[5]

def cyl_filter(input_pcd, rad_c, ht_c, neighbors):
    """ Return the filtered points that satisfy the below condition :
        * For each point, the number of nearest neighbor points specified, must lie within the cylinder defined by the point of,
            ==> radius = distance_of_the_point_in_the_point_cloud * rad_c
            ==> height = distance_of_the_point_in_the_point_cloud * tan(ht_c), where ht_c is in degrees
        Args:
            input_pcd: a string of path to the input pcd file.
            rad_c 	  : a constant value to specify the radius of the cylinder
            height 	  : a constant value in terms of degrees of vertical resolution which defines the axis of the cylinder
            neighbors : Number of nearest neighbors to classify for all the points in the point cloud
    Returns: The filtered pcd file
    """
    laser_id_dict = {'0':[],'1':[],'2':[],'3':[],'4':[],'5':[],'6':[],'7':[],'8':[],'9':[],'10':[],
                        '11':[],'12':[],'13':[],'14':[],'15':[],'16':[],'17':[],'18':[],'19':[],'20':[],'21':[],
                        '22':[],'23':[],'24':[],'25':[],'26':[],'27':[],'28':[],'29':[],'30':[],'31':[]}
    
    with open (input_pcd,"r") as pcd_file:
        lines = [line.strip().split(" ") for line in pcd_file.readlines()]
    # Retain the input pcd file headers defined in the first 11 lines of the pcd input file 
    filtered_pcd = lines[0:11]
    # Obtain the xyz_pts from the input file
    lines = lines[11:]
    lines = np.asarray(lines,dtype = np.float64)
    xyz_pts = lines[:,0:3]
    print("Total number of points are : " + str(xyz_pts.shape[0] ))
    xyz_int_id = lines[:,0:5]
    for item in range(len(xyz_pts)):
        laser_id_dict[str(int(xyz_int_id[item,4]))].append(xyz_pts[item].tolist())
    #horizontal_trees = []
    tree_list = []
    for k, val in laser_id_dict.items():
        #laser_id_dict[k].sort(key=takefirst)
        for j in laser_id_dict[k] :
            x = j[0]
            y = j[1]
            z = j[2]
            r = np.sqrt(np.square(x) + np.square(y) + np.square(z))
            j.append(r)
            zi = np.arccos(z/r)
            j.append(zi)
            theta =  np.arcsin(y/(r*np.sin(zi)))
            j.append(theta)
        laser_id_dict[k].sort(key=takefirst)
     
    set_nn = neighbors
    iter_count = 0
    filtered_count =0
    tree = cKDTree(xyz_pts) 
    dummy_flag = 0

    for i in xyz_pts:
        
        classified_pt_flag = 0
        # For each point get the distance value in spherical coordinates
        dist_from_pc = np.sqrt(np.square(i[0]) + np.square(i[1]) + np.square(i[2]))
        # Define the cylinder for a point based on its distance from the point cloud and tan of the height constant
        height = math.tan(math.radians(ht_c)) * dist_from_pc
        pt1 = []
        pt2 = []
        pt1.append(i[0])
        pt2.append(i[0])   
        pt1.append(i[1] )
        pt2.append(i[1] )
        # Define the axis of the cylinder as a line passing through the point at its mid point, def value : 1
        pt1.append(i[2] - float(height/2))
        pt2.append(i[2] + float(height/2)) 
        # Define the radius of the cylinder as a function of distance from the pc and radius const, def const used : 0.03	
        radius = (dist_from_pc*rad_c)
        # Define a vector between the plane of two circular facets of the cylinder
        vec = []
        vec.append(pt2[0] - pt1[0])
        vec.append(pt2[1] - pt1[1])
        vec.append(pt2[2] - pt1[2])
        # Constant value to confirm that the point lies inside the curved surface of the cylinder (Point-line-distance)
        const =  radius * np.linalg.norm(vec)
        count = 0
        is_inside = 0

        #Check if -set_nn and +set_nn neighbors are within the cylinder and if so, break retain and continue with next iteration
        
        test_pt = xyz_int_id[iter_count]
        test_pt_laser_id = str(int(test_pt[4]))
        test_pt_list = laser_id_dict[str(int(test_pt_laser_id))]
        #print(test_pt_laser_id)
        test_pt_list_cpy = []
        for m in range(0,len(test_pt_list)):
            test_pt_list_cpy.append(test_pt_list[m][0:3])
        #print(test_pt_list_cpy[1])
        dummy = []
        dummy.append(i[0])
        dummy.append(i[1])
        dummy.append(i[2])
        
        test_pt_pos = test_pt_list_cpy.index(dummy)
        
        for r in range(test_pt_pos-set_nn-1,test_pt_pos+set_nn-1):
            # check if a neighbor is inside the cylinder by taking a dot product of the neighbor with pt1 and pt2 and cross product to check the point-line-distance
            if r is not test_pt_pos :
                val = [test_pt_list[r][0] ,test_pt_list[r][1], test_pt_list[r][2]]
                is_inside = np.dot(np.array(val)- np.array(pt1), vec) >= 0 and np.dot(np.array(val) - np.array(pt2), vec) <= 0 and np.linalg.norm(np.cross(np.array(val) - np.array(pt1), vec)) <= const
                if(is_inside):
                    count = count + 1
                    if(count == set_nn):
                        # final_pts.append(lines[iter_count])
                        # If all neighbors lie within the cylinder, append the point to the filtered pcd list
                        filtered_pcd.append(lines[iter_count])
                        classified_pt_flag = 1
                        break   
        if not classified_pt_flag :
            filtered_count = filtered_count +1 
        iter_count = iter_count +1   
        
    print("The number of points filtered  : " + str(filtered_count))
    #print("Dummy flag count : ",dummy_flag)
    return filtered_pcd


 
def write_to_pcd_file(filtered_pts,filt_pcd_path):
    """Write the filtered pcd points to the output file
        Args:
            filtered_pts: a list of filtered pcd points
            filt_pcd_path : Path for the output filtered pcd file
        Returns: None
    """
    with open(filt_pcd_path,"w") as filtered_pcd_file :
        for i in filtered_pts:
            for j in i :
                filtered_pcd_file.write(str(j) + " ")
            filtered_pcd_file.write("\n")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("input_pcd",help = "Please provide the path to the input pcd file")
    parser.add_argument("rad_c",help = "Please provide the constant param for radius of the cylinder",type=float)
    parser.add_argument("ht_c",help = "Please provide the const param for height of the cylinder (in terms of vertical resolution of the lidar in degrees)",type=float)
    parser.add_argument("neighbors",help = "Please provide the number of nearest neighbors to classify the point on the point cloud",type=int)
    args = parser.parse_args()
    # Call the cylindrical filter with the specified constants for radius and height and number of nearest neighbors
    filt_pcd = cyl_filter(args.input_pcd,args.rad_c,args.ht_c,args.neighbors)
    # Write the filtered points to a specified path of pcd file
    filt_pcd_path = "Results/cyl_filtered_kd.pcd"
    write_to_pcd_file(filt_pcd,filt_pcd_path)
    pcd = o3d.io.read_point_cloud(filt_pcd_path)
    o3d.visualization.draw_geometries([pcd],window_name="Dynamic Cylindrical Filter")    
	

if __name__ == "__main__":
    main()

