How to run everything on ubuntu:

install qtcreator and qt5:
sudo apt-get install qt5-default
(I sugest to install qtcreator downloading it from the official website, it is more stable: https://www.qt.io/)

install these libraries:
sudo apt-get install libboost-all-dev libcgal-dev libgmp-dev libqglviewer-dev

clone libigl and create an environment variable named LIBIGL_HOME containing its directory (https://github.com/libigl/libigl/)-> eigenmesh module
install gurobi and create an environment variable named GUROBI_HOME containing its directory (https://www.gurobi.com/) 
clone cinolib and create an environment variable named CINOLIB_HOME containing its directory (https://bitbucket.org/maxicino/cinolib/overview) ->cinolib.pri


