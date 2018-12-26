1.使用命令dmesg|grep usb查看串口设备名
2.如果串口设备名为ttyUSB0，直接执行./main
  如果不是ttyUSB0，执行./main 串口设备名
3.地图保存路径需在程序中修改，默认为/mnt/hgfs/VMSHARE/mapphotos/