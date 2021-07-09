# Develop-Cylindrical-PCloud
Development of Cylindrical Surfaces in Point Cloud format

dhynn@lieutenant:~/github/Develop-Cylindrical-PCloud$ conda activate base
(base) odhynn@lieutenant:~/github/Develop-Cylindrical-PCloud$ python3 devcyl_1_fitsurf.py 

DEVELOPMENT OF CYLINDRICAL SURFACES IN POINT CLOUD FORMAT
  diploma thesis of Odysseus Galanakis
  School of Rural, Surveying and Geoinformatics Engineering
  National Technical University of Athens, 2021

  Utilizing Python 3.8
  with Open3d 0.13: A Modern Library for 3D Data Processing
               by Qian-Yi Zhou, Jaesik Park, Vladlen Koltun

First Step : Fitting the Cylinder

Open3d Version:  0.13.0

Current working directory: /home/odhynn/github/Develop-Cylindrical-PCloud
Files in the above folder may be simply specified by name.
Enter point cloud file path: Kogxi_Ierou_Mesh_Trimmed.ply

Loading 'Kogxi_Ierou_Mesh_Trimmed.ply' point cloud...
Loaded in 1.9254 sec.

Kogxi_Ierou_Mesh_Trimmed.ply : PointCloud with 8375491 points.
[[ 986.1174 1012.7862   99.9228]
 [ 986.9636 1012.8925   99.9215]
 [ 986.9627 1012.892    99.9217]
 ...
 [ 985.3237 1011.743    99.9461]
 [ 985.3239 1011.7431   99.9461]
 [ 985.3239 1011.7432   99.9461]] 

Maximum xyz: [ 987.3254 1013.4197  105.3195]
Minimum xyz: [ 984.6466 1010.8058   99.9215]

Define Initial Parameters:
Xk, x coordinate of the cylinder centre (m) = 1000
Yk, y coordinate of the cylinder centre (m) = 1000
omega, rotation angle about the x-axis (rad) = 0
phi, rotation angle about the y-axis (rad) = 0
r, radius of the cylinder (m) = 1

Initial Parameters:  [1000 1000    0    0    1]

Fitting cylinder... 
Fitted in 66.0233 sec.

Estimated parameters: 
xk = 979.3616589340986 m
yk = 1017.8708288172177 m
roll = 0.05378159930294736 rad
pitch = 0.06212967553459921 rad
cylinder radius = 0.9105948341867885 m

Cylinder Rotation Matrix:
 [[ 0.9981  0.0033  0.062 ]
 [ 0.      0.9986 -0.0538]
 [-0.0621  0.0537  0.9966]]

Calculating point distances from cylinder axis,
(may take several minutes)...
[[0.6196]
 [1.4441]
 [1.4432]
 ...
 [0.7797]
 [0.7795]
 [0.7795]]
Completed in 347.6483 sec.

Calculating point distances from cylinder radius...
[[0.291 ]
 [0.5335]
 [0.5326]
 ...
 [0.1309]
 [0.1311]
 [0.1311]]

Max. point distance from surface:
 0.9097290816987008 m
Apply distance threshold to crop off outliers?
Enter (y) to crop, any other key to continue: y
Selection Threshold
Define max. point distance to cyl. surface (m): .4

Recalibrate cylinder on cropped cloud? (y/n)
Enter (y) to continue, (n) to redefine threshold: y

Repeating cylinder fitting on the new cloud...
[[ 986.1174 1012.7862   99.9228]
 [ 986.1168 1012.7858   99.9236]
 [ 986.0483 1012.7838   99.9238]
 ...
 [ 985.3237 1011.743    99.9461]
 [ 985.3239 1011.7431   99.9461]
 [ 985.3239 1011.7432   99.9461]] 

Maximum xyz: [ 987.1793 1013.4197  105.3002]
Minimum xyz: [ 984.6466 1010.9251   99.9228]

Define Initial Parameters:
Xk, x coordinate of the cylinder centre (m) = 1000
Yk, y coordinate of the cylinder centre (m) = 1000
omega, rotation angle about the x-axis (rad) = 0
phi, rotation angle about the y-axis (rad) = 0
r, radius of the cylinder (m) = 1

Initial Parameters:  [1000 1000    0    0    1]

Fitting cylinder... 
Fitted in 29.6216 sec.

Estimated parameters: 
xk = 980.1632431697354 m
yk = 1016.9332337470461 m
roll = 0.04483370826509805 rad
pitch = 0.054503547764166986 rad
cylinder radius = 0.9583449437253447 m

Cylinder Rotation Matrix:
 [[ 0.9985  0.0024  0.0544]
 [ 0.      0.999  -0.0448]
 [-0.0545  0.0448  0.9975]]

Calculating point distances from cylinder axis,
(may take several minutes)...
[[0.6081]
 [0.6074]
 [0.5509]
 ...
 [0.7581]
 [0.7579]
 [0.7579]]
Completed in 300.1244 sec.

Calculating point distances from cylinder radius...
[[0.3502]
 [0.351 ]
 [0.4074]
 ...
 [0.2003]
 [0.2004]
 [0.2005]]

Max. point distance from surface:
 0.4527607400431207 m
Apply distance threshold to crop off outliers?
Enter (y) to crop, any other key to continue: y
Selection Threshold
Define max. point distance to cyl. surface (m): .2

Recalibrate cylinder on cropped cloud? (y/n)
Enter (y) to continue, (n) to redefine threshold: n
Selection Threshold
Define max. point distance to cyl. surface (m): .21

Recalibrate cylinder on cropped cloud? (y/n)
Enter (y) to continue, (n) to redefine threshold: y

Repeating cylinder fitting on the new cloud...
[[ 986.3002 1012.7783   99.923 ]
 [ 986.3015 1012.7781   99.923 ]
 [ 986.3003 1012.7773   99.923 ]
 ...
 [ 985.3237 1011.743    99.9461]
 [ 985.3239 1011.7431   99.9461]
 [ 985.3239 1011.7432   99.9461]] 

Maximum xyz: [ 987.0566 1013.4197  105.2806]
Minimum xyz: [ 984.6466 1011.0538   99.923 ]

Define Initial Parameters:
Xk, x coordinate of the cylinder centre (m) = 986
Yk, y coordinate of the cylinder centre (m) = 1012
omega, rotation angle about the x-axis (rad) = 0
phi, rotation angle about the y-axis (rad) = 0
r, radius of the cylinder (m) = 1

Initial Parameters:  [ 986 1012    0    0    1]

Fitting cylinder... 
Fitted in 11.4365 sec.

Estimated parameters: 
xk = 980.099750454089 m
yk = 1017.0870932498698 m
roll = 0.046477608575266255 rad
pitch = 0.05522538968377905 rad
cylinder radius = 0.9767084137530524 m

Cylinder Rotation Matrix:
 [[ 0.9985  0.0026  0.0551]
 [ 0.      0.9989 -0.0465]
 [-0.0552  0.0464  0.9974]]

Calculating point distances from cylinder axis,
(may take several minutes)...
[[0.7595]
 [0.7606]
 [0.7591]
 ...
 [0.7513]
 [0.7511]
 [0.751 ]]
Completed in 255.5869 sec.

Calculating point distances from cylinder radius...
[[0.2172]
 [0.2161]
 [0.2176]
 ...
 [0.2254]
 [0.2256]
 [0.2257]]

Max. point distance from surface:
 0.252080999824694 m
Apply distance threshold to crop off outliers?
Enter (y) to crop, any other key to continue: #  
Aligned cloud saved as PointCloud-20210709-151119.ply.
Traceback (most recent call last):
  File "devcyl_1_fitsurf.py", line 133, in <module>
    log.write("xk (m): \n" + parameter_log[-1][0] + "\n")
TypeError: can only concatenate str (not "numpy.float64") to str
