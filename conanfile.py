from conan import ConanFile


class CompressorRecipe(ConanFile):
    settings = "os", "compiler", "build_type", "arch"
    generators = "CMakeToolchain", "CMakeDeps"

    def requirements(self):
        self.requires("standardprojectsettings/[>=0.1.0]@ultimaker/stable")
        self.requires("stb/cci.20200203")
        self.requires("boost/1.79.0")
        self.requires("spdlog/1.10.0")
        self.requires("fmt/8.1.1")
        self.requires("range-v3/0.12.0")
        # self.requires("llvm-openmp/12.0.1")

    def configure(self):
        self.options["boost"].header_only = True
        self.options["clipper"].shared = False
        self.options["fmt"].shared = False
        self.options["spdlog"].shared = False

    def layout(self):
        # We make the assumption that if the compiler is msvc the
        # CMake generator is multi-config
        if self.settings.get_safe("compiler") == "msvc":
            multi = False
        else:
            multi = False

        self.folders.build = "build" if multi else f"build/{str(self.settings.build_type)}"
        self.folders.generators = f"{self.folders.build}/generators"