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
docker -H MY_ROBOT.local run -it -e mode=follow --rm -v /data:/data --net=host duckietown/rh4:v1-arm32v7
```
#### Combined behavior Braitenberg vechicle
```bash
docker -H MY_ROBOT.local run -it -e mode=redgreen --rm -v /data:/data --net=host duckietown/rh4:v1-arm32v7
```
### 4. Press ctrl+c to shut down
Note: If duckiebot does not stop after shutting down (happens approximately 1 out of 20 times), run step 3 and step 4 again.
