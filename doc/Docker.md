# Using Docker to Cross-Compile
> Note: This is not a currently recommended option. If running Linux or Windows, the instructions [Linux](InstallLinux.md) or [Windows](InstallWindows.md) are more complete. 

Docker is a light-weight virtual machine with excellent cross-platform support. This allows us to run something very close to the Target OS(Beagle bone black in this case) on any desktop or notebook computer. We get the same versions of all of the libraries running on the Target machine but compile with the power of a desktop processor.

- Install Docker and Docker hub [Docker Desktop for Mac and Windows | Docker](https://www.docker.com/products/docker-desktop)
```
docker pull alejandro4siana/cross-bbb-debian
```

- cd into the file you wish to cross compile in (e.g. CORC)

```
docker run siana/cross-bbb-debian > bbbxc
chmod +x bbbxc
```
- run the command `./bbbxc make exe    ` which should create your executable ready to upload onto Target machine