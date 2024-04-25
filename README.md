# AirHokey-Robot
This project is a robot that plays air hockey.

Mainly based on:
- Ubuntu 20.04.2 LTS / Windows
- python 3.8+
- YoLoV5 


## Install
create and activate a virtual environment

```
$ virtualenv --python /usr/bin/python3.8 airhockey-env
$ source airhockey-env/bin/activate
```

or just create venv with vscode, then activate it.

```
.\.venv\Scripts\activate  
```

then

cd to the project directory
```
$ pip install -r requirements.txt
```
**↑ If this doesn't work, try installing the packages manually**  
_Main packages:_   
- numpy-1.24.4  
- matplotlib  
- opencv-python  
- torch 2.2.0
```
$ pip install numpy
$ pip install matplotlib
$ pip install opencv-python
$ pip install pandas
$ pip install torch==2.2.0 torchvision==0.17.0 torchaudio==2.2.0 --index-url https://download.pytorch.org/whl/cu121
```

**If you are using windows:**
you may meet the following error when running the test_vision.py file**  
```
Exception: cannot instantiate 'PosixPath' on your system.
```

To fix this error, you need to uncomment the frist 3 lines in test_vision.py file  
```python
import pathlib
temp = pathlib.PosixPath
pathlib.PosixPath = pathlib.WindowsPath
```

## Vision
### quick start
```
$ python test_vision.py
```

![alt text](img/image.png)