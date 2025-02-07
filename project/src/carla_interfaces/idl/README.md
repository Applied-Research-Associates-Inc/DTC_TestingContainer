# Building the idl files
- The idl files are automatically generated when running colcon build, they can be found in `./install/carla_interfaces/share/carla_interfaces`. You can also find them by running `find . -name "*.idl"` in the root directory of the `carla_interfaces` repo
- In order to build the required files you will need to install `fastddsgen` by following these instructions
```bash
mkdir -p ~/Fast-DDS/src
cd ~/Fast-DDS/src
git clone --recursive https://github.com/eProsima/Fast-DDS-Gen.git fastddsgen
cd fastddsgen
git checkout 2.x
./gradlew assemble
```
- You can then build the cpp files with the following commands
```bash
# run from the carla_interfaces/idl folder (the same location as this readme)
mkdir output
~/Fast-DDS/src/fastddsgen/scripts/fastddsgen -d output -I ./ -replace -typeros2 <Path to idl file to build>  
```

- Rename the cxx extensions to cpp
```bash
cd output
for file in *.cxx; do mv -- "$file" "${file%.cxx}.cpp"; done
```

- In the <MsgName>PubSubTypes.cpp file you may need to manually update the SetName function to whatever the ros message name is