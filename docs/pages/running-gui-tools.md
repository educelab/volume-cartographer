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
   # Launch XQuartz
   xhost +localhost
   
   # Launch the container
   docker run --env="DISPLAY=host.docker.internal:0" ghcr.io/educelab/volume-cartographer VC
   ```
   
## Linux
```shell
docker run -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix ghcr.io/educelab/volume-cartographer VC
```

If you get error `could not connect to display :<your display number>`, run this command first to allow Docker access to X before trying again:
```shell
xhost +local:docker
```
