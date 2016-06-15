Come far compilare il progetto su ubuntu:

Installare QtCreator e Qt5.
sudo apt-get install qtcreator qt5-default

Installare le seguenti librerie:
sudo apt-get install libboost-all-dev libcgal-dev libgmp-dev libqglviewer-dev


Documentazione Dcel:

Installare Doxygen:
sudo apt-get install doxygen

dalla cartella lib:
doxygen dcel_doxygen_config

verrà creata una cartella chiamata "dcel_doc" contenente html e latex della documentazione
