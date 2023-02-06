# raisim_unitree_a1

# Здесь представлен алгоритм развертки RaiSim в docker

## 1. Получение лицензии:

Необходимо получить лицензию RaiSim, которая привязывается к ID железа машины.

Требуется перейти по [ССЫЛКЕ](https://docs.google.com/forms/d/e/1FAIpQLSc1FjnRj4BV9xSTgrrRH-GMDsio_Um4DmD0Yt12MLNAFKm12Q/viewform), заполнить все поля и указать Academic license.

Полученный файл лицензии необходимо переименовать в **_activation.raisim_** и поместить в `/home/<YOUR-USERNAME>/.raisim`

Оригинальный туториал по запуску можно найти [ЗДЕСЬ](https://raisim.com/sections/Installation.html)

## 2. Клонируем репозиторий:

Клонируем репозиторий с исходниками докера в `/home/<YOUR-USERNAME>/`
```
cd /home/$USER && git clone https://gitlab.com/rl-unitree-a1/docker.git
```
## 3. Сборка докера:

В репозитории находится скрипт по установке самого докера: _**install_docker.bash**_

Выполняем скрипт установки докера:
```
bash install_docker.bash -n
```
Выполняем скрипт клонирования репозиториев и внесения в них изменений (**** Здесь происходит клонирование ключа лицензии в докер):
```
bash initial.bash
```
Выполняем скрипт сборки докера:
```
bash build_docker.sh -n
```
И выполняем скрипт запуска докера:
```
bash run_docker.sh -n
```
# Дальнешие действия выполняются исключительно в докере


## 4. Сборка проекта:

В докере в директории **_raisim_workspace_** находятся скрипты **_build.bash_** и _**rebuild.bash**_. 

При **_первой_** сборке проекта необходимо запустить скрипт **_build.bash_**, который перемещает лицензионный ключ в необходимую директорию, а также производит создание необходимых папок build и сборки проекта.

При последующих сборках необходимо запускать скрипт _**rebuild.bash**_.

## 5. Работа с проектом:

Основными рабочими директориями в докере являются:

```
/raisim_workspace/raisimLib/raisimGymTorch/data - Местонахождение обученных моделей
/raisim_workspace/raisimLib/raisimGymTorch/raisimGymTorch/env/envs/rsg_a1 - Местонахождение исполняемых файлов
```

### 5.1 Запуск обучения


Запуск рабочего кода осуществляется в директории `/raisim_workspace/raisimLib/raisimGymTorch/raisimGymTorch/env/envs/rsg_a1`.

Для осуществления запуска обучения необходимо выполнить:
```
cd /raisim_workspace/raisimLib/raisimGymTorch/raisimGymTorch/env/envs/rsg_a1
python3 runner.py
```
В этой же директории основным файлом, где производятся все изменения и настройка симуляции является _**Environment.hpp**_.

В файле _**cfg.yaml**_ выставляются коэффициенты наказаний и наград.

### 5.2 Переобучение политики

Для осуществления запуска переобучения политики необходимо выполнить:
```
cd /raisim_workspace/raisimLib/raisimGymTorch/
python3 raisimGymTorch/env/envs/rsg_a1/runner.py --mode retrain --weight data/uni_locomotion/***/full_**.pt
```

### 5.3 Воспроизведение обученной политики

Для осуществления запуска готовой политики необходимо выполнить:
```
cd /raisim_workspace/raisimLib/raisimGymTorch/
python3 raisimGymTorch/env/envs/rsg_a1/tester.py --weight data/uni_locomotion/***/full_**.pt
```
