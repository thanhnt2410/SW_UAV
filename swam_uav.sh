cd ~/PX4-Autopilot



# Instance 0 - PX4 tự spawn x500_0

PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL=x500 \

PX4_GZ_MODEL_POSE="0,0,0.3,0,0,0" \

./build/px4_sitl_default/bin/px4 -i 0 -d &

sleep 10



# Instance 1-5 - PX4 tự spawn thêm vào world đang chạy

for i in 1 2 3 4 5; do

    PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL=x500 \

    PX4_GZ_MODEL_POSE="$((i*3)),0,0.3,0,0,0" \

    ./build/px4_sitl_default/bin/px4 -i $i -d &

    sleep 5

done