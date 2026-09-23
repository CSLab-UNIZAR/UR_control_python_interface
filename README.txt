AUTOMATIC INSTALATION:

sudo bash install.sh

MANUAL UBUNTU INSTALATION:

sudo apt install python3.10-venv
sudo apt install evtest
sudo apt-get install libhidapi-dev
sudo apt install ultraleap-hand-tracking-service ultraleap-hand-tracking-control-panel

Create a virtual environment (inside this directory):

python3 -m venv .venv

source ./.venv/bin/activate

pip install --upgrade pip

Install leap python bindings:

git clone https://github.com/ultraleap/leapc-python-bindings.git
cd leapc-python-bindings/
pip install -r requirements.txt
python -m build leapc-cffi
pip install leapc-cffi/dist/leapc_cffi-0.0.1.tar.gz
pip install -e leapc-python-api
cd ..

And then install the remaining packages:

pip install -r requirements.txt

EXECUTION:

source .venv/bin/activate

python3 -m controllers.force_controller
python3 -m controllers.key_controller
python3 -m controllers.leap_controller
python3 -m controllers.spacemouse_controller

TIPS:

In the leap controller, the aid mode moves the robot in the axis that is moving the most, the other mode, 
just replicates the movement.

In the spacemouse controller to disable the mouse movement (so it does not collide with the program)
Open a new terminal and execute 

cat /proc/bus/input/devices

In this command search where it says spacemouse and remember the event number.
Then execute (replace 20 with the correct event number):

sudo evtest --grab /dev/input/event20

If there is a problem with the permissions of the spacemouse execute in a terminal:

sudo chmod 666 /dev/hidraw*


