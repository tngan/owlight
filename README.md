# owlight

### Kickstart

```bash
$ roslaunch owlight_node joystick.launch
$ roslaunch owlight_node simulator.launch
```

Run the core ros node

```
$ chmod +x scripts/*.py
$ rosrun owlight_node <any>.py
```

### Troubleshooting

#### No module named <?>

`$ pip install <?>`

#### Cannot import cv2

`$ sudo apt-get install ros-indigo-opencv3`

#### Fix pylint cannot link with numpy and cv2 problem (temp.)

In vscode, set module whitelist in user setting.  

```
{
  "python.linting.pylintArgs": ["--extension-pkg-whitelist=numpy, cv2"]
}
```