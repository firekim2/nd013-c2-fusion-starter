# Writeup: Track 3D-Objects Over Time - midterm

## Result
Result seems like this:
![alt text](media/result.png)

precision = 0.9352750809061489, recall = 0.9444444444444444


### Range image
![alt text](media/range_image.png)

### PCL Images
![alt text](media/pcl1.png)
![alt text](media/pcl2.png)
![alt text](media/pcl3.png)
![alt text](media/pcl4.png)
![alt text](media/pcl5.png)
![alt text](media/pcl6.png)
![alt text](media/pcl7.png)
![alt text](media/pcl8.png)
![alt text](media/pcl9.png)
![alt text](media/pcl10.png)

Identifiable common features across the visible vehicles in the point clouds are:

- Side mirrors
- Some number of side windows
- General shpae of car
- Shade after that
- Quiet dense where the car is

### Bird eye view

![alt text](media/bev1.png)

But in the case of bird-eye-view, it is hard to tell there is some features like windows or tires kind of features shown.

![alt text](media/bev2.png)

When we concentrate on using BEV only in here, we are actually concentrating on 'height', 'intensity', 'dense'. Which means it doesn't directly give you 'shape' of the some features of the car, but instead it gives pattern which is detectable in numbers.

