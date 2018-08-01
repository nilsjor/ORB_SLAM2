echo "Configuring and building Thirdparty/DBoW2 ..."

cd Thirdparty/DBoW2
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j

cd ../../g2o

echo "Configuring and building Thirdparty/g2o ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j

cd ../../

echo "Configuring and building Thirdparty/Pangolin ..."

git clone https://github.com/stevenlovegrove/Pangolin.git
mkdir Pangolin/build
cd Pangolin/build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j

cd ../../../Vocabulary

echo "Uncompress vocabulary ..."

tar -xf ORBvoc.txt.tar.gz
cd ..

echo "Configuring and building ORB_SLAM2 ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j
