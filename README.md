# Instructions to reproduce results

### 1. Clone this repository and go to its directory
```bash
git clone https://github.com/splionar/rh4.git
cd rh4
```
### 2. Build docker image in Duckiebot. Make sure you are connected to your Duckiebot (able to ping).
```bash
dts devel build -f --arch arm32v7 -H MY_ROBOT.local
```

### 3. Run docker image in Duckiebot with the following options
#### Avoiding braitenberg vehicle
```bash
docker -H MY_ROBOT.local run -it -e mode=avoid --rm -v /data:/data --net=host duckietown/rh4:v1-arm32v7
```
#### Attracting braitenberg vehicle
```bash
docker -H MY_ROBOT.local run -it -e mode=attract --rm -v /data:/data --net=host duckietown/rh4:v1-arm32v7
```
#### Combined behavior Braitenberg vechicle
```bash
docker -H MY_ROBOT.local run -it -e mode=mixed --rm -v /data:/data --net=host duckietown/rh4:v1-arm32v7
```
### 4. Press ctrl+c to shut down
Note: If duckiebot does not stop after shutting down (happens approximately 1 out of 20 times), run step 3 and step 4 again.

# Changing LED Color
### 1. Build a docker image on top of duckietown/dt-duckiebot-interface:daffy-arm32v7 and run the following command
As I already built the required image before (splionar/colordetector), I used it for this task. You can also use the same image.
```bash
docker -H MY_ROBOT.local run -it -e ROS_MASTER_URI=http://[MY_ROBOT_IP]:11311/ -e ROS_IP=http://[MY_LAPTOP_IP]:11311/ splionar/colordetector /bin/bash
```
If splionar/colordetector image is not installed in your local machine, the command above should automatically look for the image in DockerHub repository and pull it first. Alternatively, you can pull the image first by running: ```
docker pull splionar/colordetector```, then execute the command above.

### 2. Run rosservice call command as stated below. HOSTNAME is normally your duckiebot name.
#### Changing to red LED
```bash
rosservice call /HOSTNAME/led_emitter_node/set_pattern "pattern_name: {data: RED}"
```
#### Changing to green LED
```bash
rosservice call /HOSTNAME/led_emitter_node/set_pattern "pattern_name: {data: GREEN}"
```

More details: http://rosapi.duckietown.p-petrov.com/repositories/dt-duckiebot-interface/docs/source/packages/led_emitter.html#ledemitternode
