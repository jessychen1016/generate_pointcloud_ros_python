# What has already been done:
* roughly trained detection of the overhead view
* set up the yolov3 detector in ROS node
* publish the bondingbox location of every detected object together with the subscribed img 
* convert the depth map and the color(mono) map to the pointcloud and publish it
* convert the depth map and the color map of the only the RoI part to the point cloud
* publsih the x, y, z locations of every point in every detected object in the custom message
* extract the x, y, z location of the same object to unify it into one (x, y, z) coordinate and plot object via matplotlib(tricky to use)



# next target:
* fit surface of the objects location
* project the objects to the fitted plane for the next matching step
* extract features with each of the three points and save it to the database
* match the latest detected feature with the database