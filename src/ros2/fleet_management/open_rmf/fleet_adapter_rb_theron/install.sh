sudo apt-get install ros-humble-rosbridge-server -y
sudo apt install ros-humble-nav2-msgs
pip install roslibpy
sudo apt update && sudo apt install curl
curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.37.2/install.sh > install_nvm.sh
. install_nvm.sh
rm install_nvm.sh
export NVM_DIR="$HOME/.nvm"
[ -s "$NVM_DIR/nvm.sh" ] && \. "$NVM_DIR/nvm.sh"  # This loads nvm
[ -s "$NVM_DIR/bash_completion" ] && \. "$NVM_DIR/bash_completion"  # This loads nvm bash_completion
source /home/robast/.bashrc
nvm install 16
curl -fsSL https://get.pnpm.io/install.sh > install_pnpm.sh
. install_pnpm.sh
rm install_pnpm.sh
source /home/robast/.bashrc
pnpm env use --global 16
sudo apt install python3-venv -y
pip3 install pipenv
export PATH=$PATH:/home/robast/.local/bin
source ~/.bashrc