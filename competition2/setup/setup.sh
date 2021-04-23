# sudo add-apt-repository ppa:alex-p/tesseract-ocr -y
# sudo apt update
# sudo apt install tesseract-ocr -y
# tesseract --version
# pip3 install pillow
# pip3 install pytesseract
# pip3 install imutils

cp ~/catkin_ws/mybash/bashrc ~/.bashrc

echo 'alias la="roslaunch competition2 start_world.launch"'  >> ~/.bashrc
echo 'alias lb="roslaunch competition2 start_robot.launch"' >> ~/.bashrc
echo 'alias ..="cd .."' >> ~/.bashrc
echo 'alias ...="cd ../.."' >> ~/.bashrc
echo 'alias src="cd ~/catkin_ws/src/competition2/src"' >> ~/.bashrc
echo 'alias launch="cd ~/catkin_ws/src/competition2/launch"' >> ~/.bashrc
echo 'alias project="cd ~/catkin_ws/experimental-mobile-robotics/competition2/src"' >> ~/.bashrc
echo 'alias update="cp -r ~/catkin_ws/src/competition2/* ~/catkin_ws/experimental-mobile-robotics/competition2/"' >> ~/.bashrc
echo 'alias copysetup="cp ~/catkin_ws/src/competition2/setup/setup.sh ~/setup.sh"'  >> ~/.bashrc
echo 'alias s="git status"' >> ~/.bashrc
echo 'alias commit="git commit -m"' >> ~/.bashrc
echo 'alias push="git push origin sheila"' >> ~/.bashrc
echo 'alias pull="git pull origin sheila"' >> ~/.bashrc
echo 'function fgit() {
  git config --global user.email "sschoepp@ualberta.ca";
  git config --global user.name "Sheila Schoepp";
  git add -A;
  git commit -m "$1";
  git push origin sheila;
}' >> ~/.bashrc

exec bash
