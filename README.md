# URDF to DH Parameterization

Sometimes people want DH... ```¯\_(ツ)_/¯```

## General Description

Each link in a URDF is connected by joint elements. The joint elements describe a homogeneous transformation that will take coordinates in the child link's coordinate frame and transform them into the parent link's coordinate frame. The joint elements also give the axis of the joint with respect to the origin of the child link's coordinate frame.

So, from the URDF we can construct a transform from any link's coordinate frame to the world coordinate frame. The root link of the URDF is assumed to be coincident with the world frame, and the same assumption is made for calculating the DH frames.

With the proceeding in mind, let [L_i] be the homogeneous transform from an arbitrary link's coordinate frame to the world coordinate frame described by URDF. Let [D_i-1] describe the DH transformation from the link's parent coordinate frame to the world coordinate frame. Then, we can find a transform [T] that describes the transformation from the link's URDF coordinate frame into the parent's DH frame: [L_i] = [T][D_i-1] and [T] = inv([D_i-1])[L_i].

Since URDF and DH frames are only relative to the parent frame, we can now find the DH parameters that take us from [D_i] to [D_i-1]. After transforming the joint axis into the parent frame, the parent link's joint axis and the current joint axis will fall into one of the following cases:
1. collinear
2. parallel
3. intersect at a single point
4. skew

### Collinear

In the collinear case the z-axis of the parent link's frame and the child link's frame are already aligned. So the DH parameters become:
* d = T[2,3]
* theta = 0 (choice of common normal is arbitrary)
* r = 0
* alpha = 0

### Parallel

In the parallel case, the z-axes of the parent and child links are already aligned, so the common normal vector is simply <T[0,3], T[1,3], 0> and the DH parameters become:
* d = T[2,3]
* theta = atan2(T[1,3], T[0,3])
* r = sqrt(T[0,3]**2, T[1,3]**2)
* alpha = 0

### Intersection


## Running the node

```
ros2 run urdf_to_dh generate_dh --ros-args -p urdf_file:="/home/andy/ros2/reference_ws/src/maxar_robot_descriptions/zeus/zeus_flight_description/urdf/zeus_flight.urdf"
```
