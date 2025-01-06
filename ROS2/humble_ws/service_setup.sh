#!/bin/bash

#! (1) create a system service 

#sudo nano /etc/systemd/system/robotController.service

#! (2) contents should look similar to the following 

#[Unit]
#Description=Robot Controller Service 
#After=network-pre.target 
# 
#[Service]
#ExecStart=/bin/bash /home/pi/Desktop/heroswarmv2/ROS2/humble_ws/startup_script.sh
#WorkingDirectory=/home/pi
#StandardOutput=file:/tmp/robotController_output.log
#StandardError=file:/tmp/robotController_error.log
#Restart=always
#User=pi
#Group=pi
#
#[Install]
#WantedBy=multi-user.target

#! (3) setup

#! Reload Systemd
#sudo systemctl daemon-reload
#! Enable the service to start on boot 
#sudo systemctl enable robotController.service
#! Start the service immediately
#sudo systemctl start robotController.service 
#! to check the status 
#sudo systemctl status robotController.service
#! to stop the service 
#sudo systemctl stop robotController.service