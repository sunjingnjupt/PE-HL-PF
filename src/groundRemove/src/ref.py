def get_slice_layer(points):
    # input: points [?, 4]
    # output: slice layer [400,352,64]   ----> target shape is:[2,64,400,352]
    points = crop_lidar(points)
    x, y, z = points[:,0],points[:,1],points[:,2]
    d = np.sqrt(x ** 2 + y ** 2) 
    #angle = np.arctan2(z + 3, d)/np.pi*180
    angle_without_three = np.arctan2(z, d) / np.pi * 180

    #angle_start = -24.8
    angle_start = -24.8723
    angle_end = 2
    v_beam_idx = np.rint((angle_without_three - angle_start) / 0.41).astype(np.int32)

    slice_layer = np.zeros(shape = (1,64,800,704),dtype = np.float32)
    x_pos,y_pos = np.rint(x / 0.1).astype(np.int32), np.rint((y + 40) / 0.1).astype(np.int32)
    z_pos = np.clip(v_beam_idx,a_min =0,a_max = 63)
    y_pos = np.clip(y_pos,a_min = 0,a_max = 799)
    x_pos = np.clip(x_pos,a_min = 0,a_max = 703)
    slice_layer[0,z_pos ,y_pos ,x_pos] = z
    return slice_layer

def change_r(points):
    # input: points [?, 4]
    # output: slice layer [400,352,64]   ----> target shape is:[2,64,400,352]
    points = crop_lidar(points)
    x, y, z = points[:,0],points[:,1],points[:,2]
    d = np.sqrt(x ** 2 + y ** 2) 
    #angle = np.arctan2(z + 3, d)/np.pi*180
    angle_without_three = np.arctan2(z, d) / np.pi * 180

    #angle_start = -24.8
    angle_start = -24.8723
    angle_end = 2
    v_beam_idx = np.rint((angle_without_three - angle_start) / 0.41).astype(np.int32)
    points[:,3] = v_beam_idx

    return points


def add_r(points):
    # input: points [?, 4]
    # output: points [?,5] add r
    points = crop_lidar(points)
    x, y, z = points[:,0],points[:,1],points[:,2]
    d = np.sqrt(x ** 2 + y ** 2) 
    #angle = np.arctan2(z + 3, d)/np.pi*180
    angle_without_three = np.arctan2(z, d) / np.pi * 180

    #angle_start = -24.8
    angle_start = -24.8723
    angle_end = 2
    v_beam_idx = np.rint((angle_without_three - angle_start) / 0.41).astype(np.int32)
    #set_trace()
    points = np.hstack([points,v_beam_idx.reshape(-1,1)])
    return points