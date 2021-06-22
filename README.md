# Point Cloud Segmentation Using PCL Library

This task is related to segmenting four specific object from a 3D Image (*.pcd) file.
Here we have used "Region Growing RGB clustering Algorithm" to detect the objects of interest from input pcd file. This particular algorithm is being used as Input pcd contains objects with different structures and also it is fast.

Following operations have been performed in this algorithm:
1. Read input pcd file and initialize PointCloud.
2. Using BoxFiltering method, remove the unused Data points from the dataset.
3. Create Segmentation Object and initialize metadata to filter the noise. Calculate Normals to implement SACSegmentaionWithNormals Algorithm.
4. We used Sample Consensus(SAC) Model and Random Sample Consensus Method for segmentation.
   In segmentation we tried to reach to the optimum value for distance threshold. We tried from 0.01 till 0.30 range. At 0.024 distance threshold we saw the better result, basically were able to reduce pcd's with noisy data.

5. Run Region Growing RGB algorithm to identify nad segment the object of interest.
6. Use PCL Visualiser to generate he procesed pcd file.


## Steps to run PointCloudSegmentation:
```
$ cd pointcloud-segmentation-poc
$ mkdir build
$ cd build
$ ckmake ..
$ make
$ ./pcs_exe
```
Input:

![image](https://user-images.githubusercontent.com/31473347/122730988-b19bb680-d22f-11eb-98ff-c8024de883e0.png)

Output:

![image](https://user-images.githubusercontent.com/31473347/122729055-9af46000-d22d-11eb-95f4-33ec10b37949.png)
