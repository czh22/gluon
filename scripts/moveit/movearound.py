#!/home/s/miniconda3/envs/gluon/bin/python3
#!coding=utf-8



import numpy as np
import dual_moveitCommander
import time

import rospy




if __name__ =="__main__":
    # User options (change me)
# --------------- Setup options ---------------
    rospy.init_node("movearound", anonymous=True)
    # workspace_limits = np.asarray([[0.3, 0.75], [0.05, 0.4], [-0.2, -0.1]]) # Cols: min max, Rows: x y z (define workspace limits in robot coordinates)
    # workspace_limits = np.asarray([[0.35, 0.55], [0, 0.35], [0.15, 0.25]])
    workspace_limits = np.asarray([[-0.2, -0.05], [-0.30, -0.15], [-0.05, 0.05]])

    calib_grid_step = 0.05 #0.05
    #checkerboard_offset_from_tool = [0,-0.13,0.02]  # change me!
    checkerboard_offset_from_tool = [0,0.121,0]
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
    print(num_calib_grid_pts-8)
    calib_grid_x.shape = (num_calib_grid_pts,1)
    calib_grid_y.shape = (num_calib_grid_pts,1)
    calib_grid_z.shape = (num_calib_grid_pts,1)
    calib_grid_pts = np.concatenate((calib_grid_x, calib_grid_y, calib_grid_z), axis=1)
    calib_grid_pts = calib_grid_pts[:-8]
    num_calib_grid_pts = num_calib_grid_pts - 8
    print(calib_grid_pts)
    moveit_server = dual_moveitCommander.MoveIt_Control()
    for calib_pt_idx in range(num_calib_grid_pts):
        tool_position = calib_grid_pts[calib_pt_idx,:]
        tool_config = [tool_position[0],tool_position[1],tool_position[2],
            tool_orientation[0],tool_orientation[1],tool_orientation[2]]
        tool_config1 = [tool_position[0], tool_position[1], tool_position[2],
                    tool_orientation[0], tool_orientation[1], tool_orientation[2]]
        print(f"tool position and orientation:{tool_config1}")
        moveit_server.left_move_position(tool_config)
        time.sleep(4)  # k