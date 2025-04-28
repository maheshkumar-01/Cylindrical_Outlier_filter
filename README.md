VLP-16 Specification:
• Supports 16 channels
• ~300,000 points per second
• 360° Horizontal field of view
• 30° Vertical field of view
• Vertical Angular resolution of 2°
• Rotation Rate of 10 Hz
Calculating angle and distance between two consecutive points from the same beam :
Number of points in the point cloud from 1 beam in a given frame (1 rotation) = Total number of points per sec
No of channels x frame rate
= 300,000
16 x 10
= 1875 points per beam
Horizontal angle between two consecutive points from the same beam (A1) = Horizontal field of view
Points per beam
= 360
1875
= 0.192°
Vertical angular resolution (data from VLP-16 Data-sheet) (A2) = 2°


Distance between two consecutive points from the same beam = Horizontal angle x Distance from the laser L1
= 0.192 x R
Where R is the distance of the point from the laser L1 in spherical coordinates
Vertical distance between the point and the laser L2 = tan(vertical angular resolution bw L1 & L2)
X R
= tan 2° x R
L1’
