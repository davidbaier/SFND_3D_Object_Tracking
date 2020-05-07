FP.0: This File ;)

FP.1 Match 3D Objects:  To match the current Bounding Boxes from the current with the previous Frame iterate over all
                        the matched bounding boxes from the current Frame. In each iteration store the matches which
                        are contained inside the roi to the bounding box structure. When for a bounding box all the
                        matches iterate through them and match them to the previous Frame Bounding Boxes. In a map
                        store the boxId with the amount of matched keypoints. Get the boxId with the highest matches
                        and add them to the bbBestMatches.

FP.2  Compute TTC Lidar: Iterate over all the the Lidarpoints inside of the bounding box. To get a stable version calculate
                         the mean over the x direction off all points. The difference between the current mean and the
                         previous mean can be used to calculate the TTC.for

FP.3 Associate Keypoint Correspondences with Bounding Boxes: Get distance between matching points inside the roi and
                                                              calculate the mean. Afterwards iterate again over the
                                                              matches and remove those who are above the mean.

FP.4 Compute Camera Based TTC: Compute the distances between all matched points. Store the calculated distance inside a
                               vector whom afterwars can be sorted. From the sorted array get the median value. Do this
                               for the current and the previous frame. From those values calculate the ratio which is
                               needed to calculate the TTC.

FP.5 Performance Evaluation 1: As can be seen in the graph in FP.5 inside of measurment_final_project.odc there
                               are different Images in which the TTC measurement is higher than in the previous ones.
                               The most interessting ones are highlighted red inside the measurement_final_project.odc.
\
                               Based on the 2D Image plane with the Bounding boxes the offset could happens because
                               of the outliers which falsify the mean calculated with the x-values of each point.
                               A possible solution would be to harden the mean algorithm or reduce the threshold by whom
                               it is decided which lidar points gets selected.

FP.6 Performance Evaluation 2: All the data are contained in measurement_measurment_final_project.odc in tab FP.6
                               There is not a graph for all the Detector/Descriptor pairs. The reason for that is, since
                               there are some data points which are way off.

                               There are different points way off, so there is only an anlyze of the worst (Which have
                               the most way off points). Which are image 2,4 and 5.

                               In the pictures the detected ROI seems to vary, which allows a wider range to detect keypoints
                               which do  not directly are on the target object(car). Also different external influences
                               in the image (light, shadows, breaklight of the car) return different keypoints which
                               do not match. This leads to an lower number of matches and therefore reduces the data to
                               calculate the mean distance from. With only a few longer distances the TTC calulations is
                               way off. This is also seen with descriptor/detector combinations which only return a smaller
                               amount of keypoint matches.

                               Interesstingly to see is that mostly at the beginning of the TTC measurements
                               the difference between the images and detector/descriptor is the highest and the error
                               converges closer to 0 during the run of the measurment.

                               It would be interessting to see how the TTC would behave when two of the best detector and
                               descriptor pairs would run in parallel and the mean would have been calculated.
