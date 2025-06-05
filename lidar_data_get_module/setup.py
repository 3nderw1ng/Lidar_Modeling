from setuptools import setup, Extension
from distutils.command.build_ext import build_ext
import numpy

#python_root = r"C:\Users\32239\AppData\Local\Programs\Python\Python311"

ext_modules = [
    Extension(
        "pylidar",
        sources=["D:\\LUA\\Lidar_Modeling\\lidar_data_get_module\\pylidar.cpp"],
        include_dirs=[numpy.get_include()],  # 加入numpy头文件路径
        extra_compile_args=["-std=c++11"],
        libraries=["ws2_32"],  # Windows socket lib
        language="c++",
    )
]

setup(
    name="pylidar",
    version="0.1",
    ext_modules=ext_modules,
)
