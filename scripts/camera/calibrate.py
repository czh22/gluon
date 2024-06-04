#!/home/s/miniconda3/envs/gluon/bin/python3
#!coding=utf-8


import matplotlib.pyplot as plt
import numpy as np
import time
import cv2

from scipy import optimize  
from mpl_toolkits.mplot3d import Axes3D
from pyzbar import pyzbar


# User options (change me)
# --------------- Setup options ---------------

# workspace_limits = np.asarray([[0.3, 0.75], [0.05, 0.4], [-0.2, -0.1]]) # Cols: min max, Rows: x y z (define workspace limits in robot coordinates)
# workspace_limits = np.asarray([[0.35, 0.55], [0, 0.35], [0.15, 0.25]])
workspace_limits = np.asarray([[-0.2, -0.05], [-0.30, -0.15], [-0.05, 0.05]])

calib_grid_step = 0.05 #0.05
#checkerboard_offset_from_tool = [0,-0.13,0.02]  # change me!
checkerboard_offset_from_tool = [0,0,0.085]
tool_orientation = [0,0,-np.pi/2] # [0,-2.22,2.22] # [2.22,2.22,0]
# tool_orientation = [np.pi/2,np.pi/2,np.pi/2]
#---------------------------------------------


# Construct 3D calibration grid across workspace
print(1 + (workspace_limits[0][1] - workspace_limits[0][0])/calib_grid_step)
gridspace_x = np.linspace(workspace_limits[0][0], workspace_limits[0][1], int(1 + (workspace_limits[0][1] - workspace_limits[0][0])/calib_grid_step))
gridspace_y = np.linspace(workspace_limits[1][0], workspace_limits[1][1], int(4))
gridspace_z = np.linspace(workspace_limits[2][0], workspace_limits[2][1], int(1 + (workspace_limits[2][1] - workspace_limits[2][0])/calib_grid_step))
calib_grid_x, calib_grid_y, calib_grid_z = np.meshgrid(gridspace_x, gridspace_y, gridspace_z)
num_calib_grid_pts = calib_grid_x.shape[0]*calib_grid_x.shape[1]*calib_grid_x.shape[2]
#print(num_calib_grid_pts)
calib_grid_x.shape = (num_calib_grid_pts,1)
calib_grid_y.shape = (num_calib_grid_pts,1)
calib_grid_z.shape = (num_calib_grid_pts,1)
calib_grid_pts = np.concatenate((calib_grid_x, calib_grid_y, calib_grid_z), axis=1)
calib_grid_pts = np.delete(calib_grid_pts,[33],axis=0)
calib_grid_pts = calib_grid_pts[:39]
num_calib_grid_pts = 38
print(calib_grid_pts)
measured_pts = []
observed_pts = []
observed_pix = []

# Move robot to home pose


# robot.open_gripper()
count = 0


# Make robot gripper point upwards

#Move robot to each calibration point in workspace
print('Collecting data...')
for calib_pt_idx in range(num_calib_grid_pts):
    tool_position = calib_grid_pts[calib_pt_idx,:]
    tool_config = [tool_position[0],tool_position[1],tool_position[2],
           tool_orientation[0],tool_orientation[1],tool_orientation[2]]
    tool_config1 = [tool_position[0], tool_position[1], tool_position[2],
                   tool_orientation[0], tool_orientation[1], tool_orientation[2]]
    print(f"tool position and orientation:{tool_config1}")
    #robot.move_j_p(tool_config)
    time.sleep(2)  # k
    
    # Find checkerboard center
    count = count + 1
    if(count==34):
        count=35
    print(count)
    #camera_color_img, camera_depth_img = robot.get_camera_data()
    camera_color_img = cv2.imread('/home/s/Desktop/catkin_workspace/src/gluon/calibrate/color_image'+str(count)+'.png')
    camera_depth_img = cv2.imread('/home/s/Desktop/catkin_workspace/src/gluon/calibrate/depth_image'+str(count)+'.png',-1)
    
    gray = cv2.cvtColor(camera_color_img, cv2.COLOR_BGR2GRAY)
    ret, thresh = cv2.threshold(gray, 170, 255, cv2.THRESH_BINARY)
        # 实例化
    barcodes = pyzbar.decode(thresh)
    checkerboard_found = barcodes[0].data.decode('ascii')
        # def decode(image, barcodes):
        #     # 循环检测到的条形码
    for barcode in barcodes:
            # 提取条形码的边界框的位置
                # 画出图像中条形码的边界框
        (x, y, w, h) = barcode.rect
        cv2.rectangle(camera_color_img, (x, y), (x + w, y + h), (255, 0, 0), 5)
    if checkerboard_found == '1':
        #corners_refined = cv2.cornerSubPix(gray_data, corners, (5,5), (-1,-1), refine_criteria)

        # Get observed checkerboard center 3D point in camera space
        checkerboard_pix = [x+w/2,y+h/2]
     
        checkerboard_z = camera_depth_img[int(checkerboard_pix[1])][int(checkerboard_pix[0])]
        print(checkerboard_z)
        checkerboard_z = checkerboard_z/1000.0
        
        checkerboard_x = np.multiply(checkerboard_pix[0]-9.5650919461068702e+02,checkerboard_z/1.2591516049668753e+03)
        checkerboard_y = np.multiply(checkerboard_pix[1]-5.3460791676518249e+02,checkerboard_z/1.2578749780456230e+03)
        if checkerboard_z == 0:
            continue

        # Save calibration point and observed checkerboard center
        observed_pts.append([checkerboard_x,checkerboard_y,checkerboard_z])
        # tool_position[2] += checkerboard_offset_from_tool
        tool_position = tool_position + checkerboard_offset_from_tool

        measured_pts.append(tool_position)
        observed_pix.append(checkerboard_pix)

        # Draw and display the corners
        # vis = cv2.drawChessboardCorners(robot.camera.color_data, checkerboard_size, corners_refined, checkerboard_found)
        # vis = cv2.drawChessboardCorners(bgr_color_data, (1,1), corners_refined[12,:,:], checkerboard_found)
        # cv2.imwrite('%06d.png' % len(measured_pts), vis)
        # cv2.imshow('Calibration',vis)
        # cv2.waitKey(1000)

# Move robot back to home pose
# robot.go_home()

measured_pts = np.asarray(measured_pts)
observed_pts = np.asarray(observed_pts)
observed_pix = np.asarray(observed_pix)
world2camera = np.eye(4)

# Estimate rigid transform with SVD (from Nghia Ho)
def get_rigid_transform(A, B):
    assert len(A) == len(B)
    N = A.shape[0] # Total points
    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)
    AA = A - np.tile(centroid_A, (N, 1)) # Centre the points
    BB = B - np.tile(centroid_B, (N, 1))
    H = np.dot(np.transpose(AA), BB) # Dot is matrix multiplication for array
    U, S, Vt = np.linalg.svd(H)
    R = np.dot(Vt.T, U.T)
    if np.linalg.det(R) < 0: # Special reflection case
       Vt[2,:] *= -1
       R = np.dot(Vt.T, U.T)
    t = np.dot(-R, centroid_A.T) + centroid_B.T
    return R, t

def get_rigid_transform_error(z_scale):
    global measured_pts, observed_pts, observed_pix, world2camera, camera

    # Apply z offset and compute new observed points using camera intrinsics
    observed_z = observed_pts[:,2:] * z_scale
    observed_x = np.multiply(observed_pix[:,[0]]-9.5650919461068702e+02,observed_z/1.2591516049668753e+03)
    observed_y = np.multiply(observed_pix[:,[1]]-5.3460791676518249e+02,observed_z/1.2578749780456230e+03)
    new_observed_pts = np.concatenate((observed_x, observed_y, observed_z), axis=1)

    # Estimate rigid transform between measured points and new observed points
    R, t = get_rigid_transform(np.asarray(measured_pts), np.asarray(new_observed_pts))
    t.shape = (3,1)
    world2camera = np.concatenate((np.concatenate((R, t), axis=1),np.array([[0, 0, 0, 1]])), axis=0)

    # Compute rigid transform error
    registered_pts = np.dot(R,np.transpose(measured_pts)) + np.tile(t,(1,measured_pts.shape[0]))
    error = np.transpose(registered_pts) - new_observed_pts
    error = np.sum(np.multiply(error,error))
    rmse = np.sqrt(error/measured_pts.shape[0])
    return rmse

# Optimize z scale w.r.t. rigid transform error
print('Calibrating...')
z_scale_init = 1
optim_result = optimize.minimize(get_rigid_transform_error, np.asarray(z_scale_init), method='Nelder-Mead')
camera_depth_offset = optim_result.x

# Save camera optimized offset and camera pose
print('Saving...')
np.savetxt('camera_depth_scale.txt', camera_depth_offset, delimiter=' ')
get_rigid_transform_error(camera_depth_offset)
camera_pose = np.linalg.inv(world2camera)
np.savetxt('camera_pose.txt', camera_pose, delimiter=' ')
print('Done.')

# DEBUG CODE -----------------------------------------------------------------------------------

# np.savetxt('measured_pts.txt', np.asarray(measured_pts), delimiter=' ')
# np.savetxt('observed_pts.txt', np.asarray(observed_pts), delimiter=' ')
# np.savetxt('observed_pix.txt', np.asarray(observed_pix), delimiter=' ')
# measured_pts = np.loadtxt('measured_pts.txt', delimiter=' ')
# observed_pts = np.loadtxt('observed_pts.txt', delimiter=' ')
# observed_pix = np.loadtxt('observed_pix.txt', delimiter=' ')

# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
# ax.scatter(measured_pts[:,0],measured_pts[:,1],measured_pts[:,2], c='blue')

# print(camera_depth_offset)
# R, t = get_rigid_transform(np.asarray(measured_pts), np.asarray(observed_pts))
# t.shape = (3,1)
# camera_pose = np.concatenate((np.concatenate((R, t), axis=1),np.array([[0, 0, 0, 1]])), axis=0)
# camera2robot = np.linalg.inv(camera_pose)
# t_observed_pts = np.transpose(np.dot(camera2robot[0:3,0:3],np.transpose(observed_pts)) + np.tile(camera2robot[0:3,3:],(1,observed_pts.shape[0])))

# ax.scatter(t_observed_pts[:,0],t_observed_pts[:,1],t_observed_pts[:,2], c='red')

# new_observed_pts = observed_pts.copy()
# new_observed_pts[:,2] = new_observed_pts[:,2] * camera_depth_offset[0]
# R, t = get_rigid_transform(np.asarray(measured_pts), np.asarray(new_observed_pts))
# t.shape = (3,1)
# camera_pose = np.concatenate((np.concatenate((R, t), axis=1),np.array([[0, 0, 0, 1]])), axis=0)
# camera2robot = np.linalg.inv(camera_pose)
# t_new_observed_pts = np.transpose(np.dot(camera2robot[0:3,0:3],np.transpose(new_observed_pts)) + np.tile(camera2robot[0:3,3:],(1,new_observed_pts.shape[0])))

# ax.scatter(t_new_observed_pts[:,0],t_new_observed_pts[:,1],t_new_observed_pts[:,2], c='green')

# plt.show()