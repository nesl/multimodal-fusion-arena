Utilize ICP in order to align two pointclouds that are already fairly close to one another via manual alignment through matrices

Not sure about how well this works when the two pointclouds are differing in terms of their size and contents. For example, two separate lidar streams are going to 
have two distinct pointclouds that may make it difficult for ICP to align. May have to feed it the background subtracted, non downsampled stream in order to have it
align the two pointclouds since only the person (shared) is present? May consider bringing in some sort of alignment board with different patterns on it as well.
