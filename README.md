# Сборка проекта UAVTalk/OpenCV

При сборке заголовочные файлы загружаются с удаленной машины, необходимо
установить переменную `RASPI_ROOT_DIR`:

    cmake -DRASPI_ROOT_DIR=<rpi_dir>
