#!/bin/bash
apt-get update >> /dev/null
apt-get install openssh-server -y 
apt-get install g++ -y
apt-get install cmake -y
apt-get install git -y
#installing the mongoc dependencies and driver
apt-get install pkg-config libssl-dev libsasl2-dev -y
apt-get install apt-get install libbson-1.0

cd ~
wget https://github.com/mongodb/mongo-c-driver/releases/download/1.6.2/mongo-c-driver-1.6.2.tar.gz
tar xzf mongo-c-driver-1.6.2.tar.gz
cd mongo-c-driver-1.6.2
./configure --disable-automatic-init-and-cleanup
make
make install
cd ~ 
rm mongo-c-driver-1.6.2.tar.gz
rm -rf mongo-c-driver-1.6.2


#installing mongocxx driver - connects c++ to mongo
wget https://github.com/mongodb/mongo-cxx-driver/archive/r3.1.1.tar.gz
tar -xzf r3.1.1.tar.gz
cd mongo-cxx-driver-r3.1.1/build
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local ..
make EP_mnmlstc_core
make
make install
cd ~
rm r3.1.1.tar.gz
rm -rf mongo-cxx-driver-r3.1.1
