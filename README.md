# Monocular Visual Odometry
## Overview
This is a personal project with the objective of developing a custom implementation of Monocular Visual Odometry. As of now, existing opencv functions were used to estimate the fundamental matrix, essential matrix and pose recovery. However, over time, I hope to add my own implementations of calculating fundamental and essential matrices, along with Nister's 5 point algorithm, and cheirality checks. I hope to contribute to this library fairly regularly. I will also be generating Doxygen documentation eventually.

Inputs
------
I will be using the KITTI Odometry dataset. The ground truth poses will be used to compute absolute scale (seeing as I don't have access to actual speedometer readings :P).
I will be considering the folder structure to be similar to the KITTI_VO dataset. So if you want to use the code for yourself, you just need to change the base path as long as the internal folder structure is the same.

```
Path to folder
+-- KITTI_VO/
|   +-- poses/
|		+-- ground truth .txt files starting with 00.txt
|   +-- sequences/
|		+-- Sequence ID based folders (00/)
|			+-- image_2/
|				+-- video frames in .png format, starting with 000000.png
|			+-- image_3/
|				+-- video frames in .png format, starting with 000000.png
|			+-- calib.txt
|			+-- times.txt
```

Todo (09/2/2018)
-----------------
1. Write and Test Pose Decomposition for Essential Matrix.
2. Move Window generation to class and add destructor to close windows after end of execution.
3. Add PCL support for RANSAC.


Example code on how to use the library will be updated in the cpp file src/main.cc. 


## How to build and run the demo
* To build and run demo:
Update path to dataset in main.cc.
```
mkdir build
cd build
cmake ..
make
./vo

```
To use these functions in your own code. Add the include directory path to the compiler debugging. Include to the start of your code and call the functions:
* To use the feature tracking and refinement library:
```
#include "FeatTrack.hpp" 
```
* To use the egomotion estimation library:
```
#include "epipolar.hpp" 
```

## License

MIT License

Copyright (c) 2018 Yash Manian

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

