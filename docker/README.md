# Docker Prerequests

* https://docs.docker.com/config/containers/resource_constraints/#gpu


# Build

```bash
docker build -t sfitc-loam_velodyne loam_velodyne/
docker build -t sfitc-zed zed/
docker build -t sfitc-vlp:latest vlp/
```

# Run

```bash
xhost +si:localuser:root
```