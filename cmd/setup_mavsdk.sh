echo "===========Setting up MAVSDK==========="
cd dependencies/
rm -rf MAVSDK-Python/

git clone --branch v3.15.3 --recursive https://github.com/mavlink/MAVSDK-Python.git
cd MAVSDK-Python
cd proto/pb_plugins
cd ../..
./other/tools/run_protoc.sh
python3 setup.py build
pip3 install -e .

echo "Done"
cd ..
