José Manuel Alcalde Llergo

Calibration
Extrínseca
./calibrate -s=0.04 -r=5 -c=6 -verbose calibracion.yml ../data/logitech_000_000.png ../data/logitech_000_001.png
./calibrate -s=0.04 -r=5 -c=6 -verbose calibracion.yml ../data/logitech_000_000.png ../data/logitech_000_001.png ../data/logitech_000_003.png ../data/logitech_000_004.png ../data/logitech_000_005.png ../data/logitech_000_006.png ../data/logitech_000_007.png ../data/logitech_000_008.png

Íntrinseca
./calibrate -s=0.04 -r=5 -c=6 -verbose -i=../data/logitech.xml calibracion.yml ../data/logitech_000_000.png

Undistort image
./undistort ../data/elp-intrinsics.xml ../data/elp-view-001.jpg out.png

Undistort Video
./undistort -v ../data/logitech.xml ../data/tablero_000_000.avi video.avi