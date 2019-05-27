echo "start install dependency of ceres"
sudo apt-get update
sudo apt-get install libgoogle-glog-dev
sudo add-apt-repository ppa:bzindovic/suitesparse-bugfix-1319687
sudo apt-get update
sudo apt-get install libsuitesparse-dev

echo "start build and install ceres"
cd ~
git clone https://github.com/ceres-solver/ceres-solver.git ./ceres-solver
cd ceres-solver
mkdir ceres-bin
cd ceres-bin
cmake ..
make -j8
sudo make install

