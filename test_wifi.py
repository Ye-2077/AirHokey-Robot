import pathlib
from motion.kinematics import *
from wifi.CommandServer import CommandServer

if __name__ == "__main__":
    temp = pathlib.PosixPath
    pathlib.PosixPath = pathlib.WindowsPath
    server = CommandServer()
    server.run()
