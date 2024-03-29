device_imu_inertialsense
==============
This is the Ubitrack device_imu_inertialsense submodule.

Description
----------
The device_imu_inertialsense contains components for imu data aquisition using the InertialSenseSDK.

## For Users: Use this package

### Project setup

If you handle multiple dependencies in your project is better to add a *conanfile.txt*

    [requires]
    ubitrack_device_imu_inertialsense/1.3.0@ubitrack/stable

    [generators]
    cmake
    txt

Complete the installation of requirements for your project running:

    $ mkdir build && cd build && conan install ..
    
Note: It is recommended that you run conan install from a build directory and not the root of the project directory.  This is because conan generates *conanbuildinfo* files specific to a single build configuration which by default comes from an autodetected default profile located in ~/.conan/profiles/default .  If you pass different build configuration options to conan install, it will generate different *conanbuildinfo* files.  Thus, they shoudl not be added to the root of the project, nor committed to git. 

## For Packagers: Publish this Package

The example below shows the commands used to publish to ulricheck conan repository. To publish to your own conan respository (for example, after forking this git repository), you will need to change the commands below accordingly. 

## Build and package 

The following command both runs all the steps of the conan file, and publishes the package to the local system cache.  This includes downloading dependencies from "build_requires" and "requires" , and then running the build() method. 

    $ conan create . ubitrack/stable
    
## Add Remote

    $ conan remote add camp "https://conan.campar.in.tum.de" True

## Upload

    $ conan upload -r camp ubitrack_device_imu_inertialsense/1.3.0@ubitrack/stable
