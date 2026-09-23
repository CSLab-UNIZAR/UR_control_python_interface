#!/bin/bash

# Colors for output
GREEN="\e[32m"
RED="\e[31m"
BLUE="\e[34m"
YELLOW="\e[33m"
RESET="\e[0m"

if [ "$EUID" -ne 0 ]; then
    echo -e "${RED}Please run this script as root.${RESET}"
    exit
fi

echo -e "${YELLOW}==============================================${RESET}"
echo -e "${YELLOW}      UR_CONTROL INSTALLER (by Adrian)        ${RESET}"
echo -e "${YELLOW}==============================================${RESET}"

echo -e "${BLUE}Cheking if python is installed...${RESET}"

if ! command -v python3 &> /dev/null; then
    echo -e "${RED}Python3 is not installed. Installing...${RESET}"
    apt update
    if apt install -y python3; then
        echo -e "${GREEN}Python3 has been installed successfully.${RESET}"
    else
        echo -e "${RED}Failed to install Python3.${RESET}"
        exit 1
    fi
else
    echo -e "${GREEN}Python3 is already installed.${RESET}"
fi

echo -e "${BLUE}Cheking if git is installed...${RESET}"

if ! command -v git &> /dev/null; then
    echo -e "${RED}Git is not installed. Installing...${RESET}"
    apt update
    if apt install -y git; then
        echo -e "${GREEN}Git has been installed successfully.${RESET}"
    else
        echo -e "${RED}Failed to install Git.${RESET}"
        exit 1
    fi
else
    echo -e "${GREEN}Git is already installed.${RESET}"
fi

echo -e "${BLUE}Installing python virtual environments...${RESET}"

apt install python3-venv python3-pip build-essential python3-dev python3-build python3-wheel git -y

echo -e "${BLUE}Installing libraries for the spacemouse...${RESET}"

apt install evtest -y
apt-get install libhidapi-dev -y

echo -e "${BLUE}Installing ultraleap (for the leap motion)...${RESET}"

wget -qO - https://repo.ultraleap.com/keys/apt/gpg | gpg --dearmor | sudo tee /etc/apt/trusted.gpg.d/ultraleap.gpg
echo 'deb [arch=amd64] https://repo.ultraleap.com/apt stable main' | sudo tee /etc/apt/sources.list.d/ultraleap.list
apt update
apt install ultraleap-hand-tracking-service ultraleap-hand-tracking-control-panel -y

sudo -u "$SUDO_USER" bash <<EOF

echo -e "${BLUE}Creating virtual environment...${RESET}"

python3 -m venv .venv

echo -e "${BLUE}Activating virtual environment...${RESET}"

source .venv/bin/activate
pip install --upgrade pip

echo -e "${BLUE}Installing leap python bindings...${RESET}"

git clone https://github.com/ultraleap/leapc-python-bindings.git
cd leapc-python-bindings/
pip install -r requirements.txt
python -m build leapc-cffi
pip install leapc-cffi/dist/leapc_cffi-0.0.1.tar.gz
pip install -e leapc-python-api
cd ..

echo -e "${BLUE}Installing the remaining python packages...${RESET}"

pip install -r requirements.txt

echo -e "${GREEN}Installation completed successfully!${RESET}"

EOF