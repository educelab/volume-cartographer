# Running the VC GUI tools from the Docker container

## Windows
1. Install VcXsrv or an equivalent X Server
2. Launch the X Server and give it permission to access connections from all clients
3. Launch the GUI app from the Docker container with appropriate X11 forwarding:
   ```shell
   docker run --env="DISPLAY=host.docker.internal:0" ghcr.io/educelab/volume-cartographer VC
   ```

## macOS
1. Install [XQuartz](https://www.xquartz.org/)
   ```shell
   brew install --cask xquartz
   ```
2. Launch XQuartz. In the Preferences/Security tab, enable "Allow connections from network clients"
3. Restart XQuartz
4. Launch the GUI app from the Docker container with appropriate X11 forwarding:
   ```shell
   # Give Docker permission to access the X server (will also launch XQuartz)
   # Needs to be run every time the XQuartz process is launched
   xhost +localhost
   
   # Launch the container
   docker run --env="DISPLAY=host.docker.internal:0" ghcr.io/educelab/volume-cartographer VC
   ```
   
## Linux
```shell
# Give Docker permission to access the X server
# Needs to be run every time the X server is relaunched (e.g. after reboots)
xhost +local:docker

docker run -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix ghcr.io/educelab/volume-cartographer VC
```

**Note:** On some distributions, you may need to run Docker with superuser 
privileges: `sudo docker ...`

## FAQ
### Qt complains that it can't connect to the display
If you get error `could not connect to display <your display number>`, make 
sure you run the following command to give Docker access to the X server:

```shell
# macOS: needs to be run every time the XQuartz process is launched
xhost +localhost

# Linux: needs to be run every time the X server is relaunched (e.g. after reboots)
xhost +local:docker
```

### I've done that, but Qt still can't connect to the display

Are you using the Docker snap package? We've learned the hard way that the 
snap sandbox makes it extremely difficult to forward X11 windows to the host 
system. We recommend 
[installing Docker Engine with apt](https://docs.docker.com/engine/install/)
(or the equivalent for your flavor of Linux). If you absolutely need to use 
the snap package, or still have issues connecting to the display, you may be 
interested in [x11docker](https://github.com/mviereck/x11docker).