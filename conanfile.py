from conans import ConanFile, CMake, tools

import os
print(os.getenv("SHORTVER", 0.1))
ver = str(os.getenv("SHORTVER", 0.1))

class SLAMdnkConan(ConanFile):
    name = "SLAMdnk"
    version = ver
    license = "<Put the package license here>"
    author = "KSG"
    url = "<Package recipe repository url here, for issues about the package>"
    description = "<Description of SLAMdnk here>"
    topics = ("SLAM", "ML", "NN", "SensorFusion")
    settings = "os", "compiler", "build_type", "arch"
    options = {"shared": [True, False], "fPIC": [True, False]}
    default_options = {"shared": False, "fPIC": True}
    generators = "cmake"
    export_sources = "*"

    def config_options(self):
        if self.settings.os == "Windows":
            del self.options.fPIC

    def configure(self):
        self.options["opencv"].parallel="tbb"
        # self.options["opencv"].with_contrib=True
        self.options["gtsam"].with_TBB=True

    def requirements(self):
        self.requires("eigen/3.3.9")
        self.requires("opencv/4.5.1")
        self.requires("gtsam/4.0.3")
        self.requires("tbb/2020.2", override=True)
        self.requires("spdlog/1.8.2")
        self.requires("gstreamer/1.18.0")

    def package(self):
        # self.copy("*.h", dst="include", src="src")
        # # self.copy("*hello.lib", dst="lib", keep_path=False)
        # self.copy("*.dll", dst="bin", keep_path=False)
        # self.copy("*.so", dst="lib", keep_path=False)
        # self.copy("*.dylib", dst="lib", keep_path=False)
        # self.copy("*.a", dst="lib", keep_path=False)
        self.copy("*.dll", dst="bin", src="bin")
        self.copy("*.dylib", dst="bin", src="lib")
        # self.copy("*.lib", dst="bin", src="lib")

    def package_info(self):
        self.cpp_info.libs = ["SLAMdnk"]

    def build(self):
        cmake = CMake(self)
        cmake.configure()#source_folder="src")
        cmake.build()
